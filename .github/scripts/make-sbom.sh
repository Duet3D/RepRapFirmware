#!/bin/bash
# Builds the CycloneDX software BOM for the Duet 3 firmware from a workspace holding one checkout
# per source repository. Everything that cannot be read out of git - the vendored third-party
# copies, the upstream identity of the forked repositories, licences - comes from
# sbom-components.json; versions are extracted from the sources so they cannot go stale.
#
# Usage: make-sbom.sh <workspace> <firmware version> <toolchain directory name>

set -euo pipefail

die() {
	echo "make-sbom: $*" >&2
	exit 1
}

[ $# -eq 3 ] || die "usage: $0 <workspace> <firmware version> <toolchain directory name>"

workspace=$1
firmwareVersion=$2
toolchain=$3
manifest="$(cd "$(dirname "$0")/.." && pwd)/sbom-components.json"

[ -d "$workspace" ] || die "workspace $workspace does not exist"
[ -f "$manifest" ] || die "component manifest $manifest does not exist"

# Resolves one version rule: a literal, the single versioned subdirectory of a pack directory, or
# the parts captured by a list of full-line patterns, joined with dots
resolveVersion() {
	local dir=$1 rule=$2
	local literal packDir file pattern part version
	local -a candidates

	literal=$(jq -r '.literal // empty' <<< "$rule")
	if [ -n "$literal" ]; then
		echo "$literal"
		return
	fi

	packDir=$(jq -r '.dir // empty' <<< "$rule")
	if [ -n "$packDir" ]; then
		candidates=("$dir/$packDir"/*/)
		[ -d "${candidates[0]}" ] || die "$dir/$packDir holds no version directory"
		[ ${#candidates[@]} -eq 1 ] || die "$dir/$packDir holds more than one version directory"
		basename "${candidates[0]}"
		return
	fi

	file=$(jq -r '.file' <<< "$rule")
	[ -f "$dir/$file" ] || die "version file $dir/$file is missing, the manifest is stale"
	version=""
	while IFS= read -r pattern; do
		part=$(sed -nE "s%^$pattern\$%\1%;T;p;q" "$dir/$file")
		[ -n "$part" ] || die "pattern $pattern did not match in $dir/$file, the manifest is stale"
		version="${version:+$version.}$part"
	done < <(jq -r '.patterns[]' <<< "$rule")
	echo "$version"
}

# The commit of every source repository that goes into the binaries, plus the resolved versions the
# manifest asks for. Checked-out submodules are listed as components of their parent, which today
# means the tinyusb fork linked into the firmware; recorded but uninitialised submodules contribute
# no code and are skipped
facts=$(jq -n '{repos: {}, submodules: [], versions: {}}')

while IFS= read -r repo; do
	dir="$workspace/$repo"
	[ -d "$dir/.git" ] || die "$dir is not a checkout"
	facts=$(jq --arg repo "$repo" --arg sha "$(git -C "$dir" rev-parse HEAD)" --arg ref "$(git -C "$dir" rev-parse --abbrev-ref HEAD)" '.repos[$repo] = {sha: $sha, ref: $ref}' <<< "$facts")

	rule=$(jq -c --arg repo "$repo" '.forks[$repo].ancestor.version // empty' "$manifest")
	if [ -n "$rule" ]; then
		# Resolved into a variable of its own because a die() inside a command substitution that
		# feeds an argument would only kill the subshell, leaving the version silently empty
		resolved=$(resolveVersion "$dir" "$rule")
		facts=$(jq --arg key "fork:$repo" --arg version "$resolved" '.versions[$key] = $version' <<< "$facts")
	fi

	while read -r sha path _; do
		case $sha in -*) continue;; esac
		sha=${sha#[+U]}
		name="$repo/${path#*/}"
		facts=$(jq --arg name "$name" --arg parent "$repo" --arg sha "$sha" '.submodules += [{name: $name, parent: $parent, sha: $sha}]' <<< "$facts")
		rule=$(jq -c --arg name "$name" '.submodules[$name].ancestor.version // empty' "$manifest")
		if [ -n "$rule" ]; then
			resolved=$(resolveVersion "$dir/$path" "$rule")
			facts=$(jq --arg key "submodule:$name" --arg version "$resolved" '.versions[$key] = $version' <<< "$facts")
		fi
	done < <(git -C "$dir" submodule status --recursive)
done < <(jq -r '.repos[]' "$manifest")

while IFS= read -r entry; do
	index=$(jq -r '.key' <<< "$entry")
	repo=$(jq -r '.value.repo' <<< "$entry")
	jq -e --arg repo "$repo" '.repos | index($repo)' "$manifest" > /dev/null || die "vendored component $index names unknown repository $repo"
	while IFS= read -r path; do
		[ -e "$workspace/$repo/$path" ] || die "$repo/$path is gone, the manifest is stale"
	done < <(jq -r '.value.paths[]' <<< "$entry")

	rule=$(jq -c '.value.version // empty' <<< "$entry")
	if [ -n "$rule" ]; then
		resolved=$(resolveVersion "$workspace/$repo" "$rule")
		facts=$(jq --arg key "vendored:$index" --arg version "$resolved" '.versions[$key] = $version' <<< "$facts")
	fi
done < <(jq -c '.vendored | to_entries[]' "$manifest")

jq -n --arg firmwareVersion "$firmwareVersion" --arg toolchain "$toolchain" --slurpfile manifest "$manifest" --argjson facts "$facts" '
	def purl($template; $version):
		if $template == null then null
		elif ($version // "") == "" then ($template | sub("@\\{version\\}$"; ""))
		else ($template | gsub("\\{version\\}"; $version))
		end;
	def licences($entry):
		if $entry.licenseExpression != null then {licenses: [{expression: $entry.licenseExpression}]}
		elif $entry.license != null then {licenses: [{license: {id: $entry.license}}]}
		elif $entry.licenseName != null then {licenses: [{license: {name: $entry.licenseName}}]}
		else {}
		end;
	def references($entry):
		if $entry.vcs == null then {} else {externalReferences: [{type: "vcs", url: $entry.vcs}]} end;
	def version($value):
		if ($value // "") == "" then {} else {version: $value} end;
	def pedigree($entry; $ancestorVersion):
		(if $entry.ancestor == null then {} else
			{pedigree: {ancestors: [
				{type: "library", name: $entry.ancestor.name}
				+ version($ancestorVersion)
				+ (purl($entry.ancestor.purl; $ancestorVersion) as $value | if $value == null then {} else {purl: $value} end)
				+ references($entry.ancestor)
				+ licences($entry.ancestor)
			]}}
		end)
		* (if $entry.notes == null then {} else {pedigree: {notes: $entry.notes}} end);

	$manifest[0] as $m
	| [ $m.repos[] as $repo
		| ($m.forks[$repo] // {}) as $fork
		| ({
				type: "library",
				name: $repo,
				version: $facts.repos[$repo].sha,
				purl: ("pkg:github/Duet3D/" + $repo + "@" + $facts.repos[$repo].sha),
				externalReferences: [{type: "vcs", url: ("https://github.com/Duet3D/" + $repo)}],
				properties: [{name: "duet3d:ref", value: $facts.repos[$repo].ref}]
			} + licences($fork) | . * pedigree($fork; $facts.versions["fork:" + $repo])),
		  ( $facts.submodules[] | select(.parent == $repo) | . as $submodule
			| ($m.submodules[$submodule.name] // {}) as $entry
			| ({
					type: "library",
					name: $submodule.name,
					version: $submodule.sha,
					properties: [{name: "duet3d:submoduleOf", value: $submodule.parent}]
				} + licences($entry) | . * pedigree($entry; $facts.versions["submodule:" + $submodule.name]))
		  )
	  ]
	+ [ $m.vendored | to_entries[] | .key as $index | .value as $entry
		| $facts.versions["vendored:" + ($index | tostring)] as $componentVersion
		| ({
				"bom-ref": ("vendored:" + $entry.repo + "/" + $entry.paths[0]),
				type: "library",
				name: $entry.name
			}
			+ version($componentVersion)
			+ (purl($entry.purl; $componentVersion) as $value | if $value == null then {} else {purl: $value} end)
			+ references($entry)
			+ licences($entry)
			+ {properties: [{name: "duet3d:vendoredIn", value: ($entry.paths | map($entry.repo + "/" + .) | join(", "))}]}
			| . * pedigree($entry; null))
	  ]
	| {
		bomFormat: "CycloneDX",
		specVersion: "1.6",
		version: 1,
		metadata: {
			timestamp: (now | todateiso8601),
			component: {type: "firmware", name: "RepRapFirmware", version: $firmwareVersion},
			tools: {components: [{type: "application", name: "arm-gnu-toolchain", version: ($toolchain | sub("^arm-gnu-toolchain-"; "") | sub("-x86_64-arm-none-eabi$"; ""))}]}
		},
		components: .
	  }
'
