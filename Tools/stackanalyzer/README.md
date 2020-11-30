# stackanalyzer
Small CLI tool to extract information about failure locations.

To use this tool one needs to provide the output of `M122` (at least the line starting with "Stack:")
as well as the map file created by the linker at build time. `stackanalyzer` will then search
the location the error has happened and output the information extracted from the map file.

## Usage
```
$ stackanalyzer --help
Usage of linux/stackanalyzer:
  -m122 string
        Path to file with M122 output (even partial) or "-" for stdin (default "-")
  -mapFile string
        Path to .map file
```

## Examples
### `AssertionFailed`
Note that for failures of this type the line starting with "Last software reset at " has to be present in the input.

```
$ stackanalyzer -m122 m122AssertionFailed.txt -mapFile Duet3Firmware_MB6HC.map
Faulted at:
.rodata.prvTaskExitError.str1.4
                0x004886b0        0xf C:\Eclipse\Firmware\FreeRTOS\SAME70\libFreeRTOS.a(queue.o)
Error is at line: 1433
```
The stack above showed that an assertion failed in `queue.c` located (somewhere) in project `FreeRTOS`
at line 1433. The user needs to look up that location and check which assertion failed.

### `MemoryProtectionFault`

```
$ linux/stackanalyzer -m122 m122MemoryProtectionFault.txt -mapFile ../../Duet3/Duet3Firmware_MB6HC.map
Faulted at:
.text._ZN9FileStore5StoreEPKcjPj
                0x0000000000404bb8       0xe8 ./src/Storage/FileStore.o
                0x0000000000404bb8                FileStore::Write(char const*, unsigned int)
Error is at offset: 0x5e
```
This stack analysis leads to the method `FileStore::Write()`. The user then needs to open the corrsponding
`.s` file that is usually located next to the `.map` file search for the beginning of the method (use the
first line after "Faulted at:") and then search for offset `0x5e`.

## Building
This tool is writting in Go and can be simply built by `go build stackanalyzer.go`. For convenience
there are pre-built binaries for x86_64 Windows/Linux/MacOS in sub directories.