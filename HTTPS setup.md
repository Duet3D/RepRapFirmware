# HTTPS / TLS Setup for RepRapFirmware

RRF supports TLS-encrypted protocols on boards with an Ethernet interface (e.g. Duet 3 MB6HC, MB6XD, Mini 5+) and on boards with an ESP32-family WiFi module (Duet 3 MB6HC with the optional WiFi daughterboard, running WiFi firmware 2.4.0 or later). All three selectable TCP protocols can be secured with TLS:

| Protocol | Default port | Plain command | TLS variant | Default TLS port |
|----------|-------------|--------------|------------|------------------|
| HTTP     | 80          | `M586 P0 S1` | HTTPS      | 443              |
| FTP      | 21          | `M586 P1 S1` | FTPS       | 990              |
| Telnet   | 23          | `M586 P2 S1` | TelnetS    | 992              |

You need to provide a private key and a certificate in PEM format on the SD card before enabling any TLS variant.

> **WiFi support:** TLS on WiFi requires an ESP32-family module **and** WiFi firmware version 2.4.0 or later. The ESP8266-based WiFi module used on Duet 3 Mini 5+ and FMDC cannot host a TLS server - its limited heap (~38 KiB free) cannot accommodate an mbedTLS handshake. On the WiFi path, cert/key are uploaded once to the WiFi module's flash and the SD copies are auto-deleted (see [Section 2](#2-install-the-files-on-the-sd-card) below).

> **SBC mode note:** In SBC mode, HTTPS is handled by DuetWebServer (DSF) via ASP.NET Core / Kestrel and has been supported since DSF 3.3. DSF manages its own certificate independently (`/opt/dsf/conf/https.pfx`), which it auto-generates if not present. The `server.key` and `server.crt` files described in this document are only used by the standalone Ethernet interface - they are not relevant in SBC mode.

---

## 1. Generate a private key and self-signed certificate

The examples below generate an **EC P-256** key (the only key type accepted by the TLS server on any interface) and a self-signed certificate valid for 10 years, with the board's hostname set as `duet3`.

Replace `duet3` with your board's actual hostname or IP address as appropriate.

### Linux / macOS

OpenSSL is usually pre-installed. Open a terminal and run:

```bash
# Generate EC private key
openssl ecparam -name prime256v1 -genkey -noout -out server.key

# Generate self-signed certificate (10 years)
openssl req -new -x509 \
  -key server.key \
  -out server.crt \
  -days 3650 \
  -subj "/CN=duet3" \
  -addext "subjectAltName=DNS:duet3"
```

To include an IP address SAN as well (useful if you access the board by IP):

```bash
openssl req -new -x509 \
  -key server.key \
  -out server.crt \
  -days 3650 \
  -subj "/CN=duet3" \
  -addext "subjectAltName=DNS:duet3,IP:192.168.1.2"
```

### Windows

**Option A - OpenSSL via winget**

Install OpenSSL:

```powershell
winget install -e --id ShiningLight.OpenSSL.Light
```

Then open a new PowerShell/Command Prompt window and run the same commands as Linux above.

> If winget reports "No package found matching input criteria", the package ID has changed - run `winget search openssl` and use the current Shining Light Productions ID, or fall back to Option B.

**Option B - OpenSSL bundled with Git for Windows**

If [Git for Windows](https://gitforwindows.org/) is installed, open **Git Bash** and run the same Linux commands above verbatim.

**Option C - Windows Subsystem for Linux (WSL)**

Open a WSL terminal and follow the Linux instructions.

---

## 2. Install the files on the SD card

Copy both generated files to the `/sys/` directory on the Duet SD card:

| File | Destination on SD card |
|------|------------------------|
| `server.key` | `/sys/server.key` |
| `server.crt` | `/sys/server.crt` |

> **Security note:** `server.key` is your private key. Keep it confidential - anyone with access to this file can impersonate your board.

> **WiFi path:** On WiFi-equipped boards, `M552 T1 S1` automatically imports these files to the WiFi module's flash and then **securely wipes and deletes the SD copies**. After the first successful enable, the files are persisted on the module and the SD locations are empty. To rotate the cert/key, disable the interface (`M552 S0`), drop new files in `/sys/`, then re-enable (`M552 T1 S1`); the new files take priority and the old WiFi-flash copy is replaced.

---

## 3. Enable TLS in RRF

Plain and TLS variants of each protocol are **independent** - enabling one does not affect the other. Both variants can run simultaneously on separate ports.

### Enable TLS on the network interface (M552)

Before any TLS protocol variant can be used, TLS support must be enabled on the network interface itself using `M552 T1`. The default is `T0` (TLS disabled).

```gcode
M552 T1 S1        ; enable TLS support and bring up the network interface
```

`M552 T1` causes RRF to load `server.key` and `server.crt` from the SD card. If those files are missing or invalid, the command will report an error and TLS will not be available.

To disable TLS support (revert to plain-only):

```gcode
M552 T0           ; disable TLS support (default)
```

> **Note:** `M552 T1` only enables the TLS layer. You must still use `M586` with `T1` to enable each individual TLS protocol variant (HTTPS, FTPS, TelnetS).

### M552 T parameter

- `T0` - TLS disabled (default); `server.key` and `server.crt` are not loaded
- `T1` - TLS enabled; loads the key and certificate from `/sys/server.key` and `/sys/server.crt` (Ethernet), or imports them into the WiFi module's flash and deletes the SD copies (WiFi)
- `T-1` - Clear stored TLS material and start the interface in plain mode. On Ethernet, securely deletes `/sys/server.key` and `/sys/server.crt`. On WiFi, sends a clear command that wipes the cert/key from the WiFi module's flash. The interface still comes up; only TLS variants are unavailable.

To change TLS state on a running interface: on WiFi, simply re-issue `M552 T1 S1` (or `T-1`/`T0`) - the WiFi module is probed immediately and the new state takes effect. On Ethernet, first run `M552 S0`, then `M552 T1 S1` - the LwIP TLS heap is sized at `Start()` time and a full Stop/Start cycle is needed to resize it.

> **Note:** Omitting `T` is equivalent to `T0` - the TLS state is **not** remembered across enable cycles. If you re-issue a bare `M552 S1` after TLS was previously enabled (for example to change SSID or IP settings), TLS support is turned off again. Always include `T1` whenever you bring the interface up if you want TLS, e.g. keep a single `M552 ... S1 T1` line in `config.g` rather than a later plain `M552 S1`.

---

Send the following G-code (via USB, Telnet, or the existing HTTP interface).

### HTTPS

```gcode
; Enable HTTPS on the default port 443
M586 P0 S1 T1

; Run both plain HTTP (port 80) and HTTPS (port 443) simultaneously
M586 P0 S1        ; enable plain HTTP
M586 P0 S1 T1     ; enable HTTPS (plain HTTP state unchanged)

; HTTPS on a non-standard port
M586 P0 S1 T1 R8443
```

### FTPS (FTP over TLS)

```gcode
M586 P1 S1 T1     ; enable FTPS on default port 990
```

### TelnetS (Telnet over TLS)

```gcode
M586 P2 S1 T1     ; enable TelnetS on default port 992
```

### M586 parameters

- `P` - protocol: `0` = HTTP/HTTPS, `1` = FTP/FTPS, `2` = Telnet/TelnetS
- `S1` - enable; `S0` - disable both plain **and** TLS for that protocol
- `T1` - target the TLS variant; omitting `T` (or `T0`) targets the plain variant
- `R` - port number for the selected variant (optional)

### Disabling TLS

There is no command to disable the TLS variant alone while leaving plain enabled. `S0` disables both:

```gcode
; Disable all HTTP (plain and HTTPS), then re-enable plain only:
M586 P0 S0
M586 P0 S1
```

---

## 4. Trusting the self-signed certificate

Because the certificate is self-signed (not issued by a public CA), browsers will display a security warning on first connection. You have two options:

### Add a browser exception (quick)

Click through the browser's "Advanced -> Proceed anyway" prompt. The connection is still encrypted; only certificate authenticity is unverified.

### Import the certificate into your OS trust store (recommended)

#### Linux

```bash
sudo cp server.crt /usr/local/share/ca-certificates/duet3.crt
sudo update-ca-certificates
```

#### macOS

```bash
sudo security add-trusted-cert -d -r trustRoot \
  -k /Library/Keychains/System.keychain server.crt
```

#### Windows

```powershell
Import-Certificate -FilePath .\server.crt `
  -CertStoreLocation Cert:\LocalMachine\Root
```

Or double-click `server.crt` -> **Install Certificate** -> **Local Machine** -> **Trusted Root Certification Authorities**.

---

## 5. Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| `cannot open /sys/server.crt` error message | File missing or wrong name/path on SD card |
| `cannot open /sys/server.key` error message | File missing or wrong name/path on SD card |
| `failed to create TLS config` error message | Mismatched key/certificate pair, or unsupported format - regenerate both files |
| Browser warning after importing certificate | SAN in the certificate does not match the hostname/IP you are connecting to - regenerate with the correct `-addext "subjectAltName=..."` value |
| FTPS/TelnetS not working after `M586 P1/P2 S1 T1` | The same certificate and key loaded for HTTPS is reused for all TLS protocols - ensure it was loaded successfully first |
| (WiFi) `TLS: not supported on this WiFi firmware version` | WiFi firmware is older than 2.4.0, or running on an ESP8266 module. Update WiFi firmware. |
| (WiFi) `TLS: no TLS cert/key on WiFi module flash and none in /sys/ to import` | First-time setup needs `server.crt` and `server.key` on the SD card. Place them in `/sys/` and re-run `M552 T1 S1`. |
| (WiFi) `TLS: WiFi module rejected new TLS material` | The PEM files were transferred but mbedTLS could not parse them. Regenerate the cert/key pair; ensure they match. |

---

## Notes

- TLS is handled by **mbedTLS** - via lwIP's `altcp_tls` layer on the Ethernet interface, and directly in the WiFi module firmware on the WiFi interface.
- Only PEM-encoded keys and certificates are supported. DER-encoded files will not work.
- The private key must **not** be password-protected (no passphrase).
- The server certificate **must** use an EC key on the P-256 (secp256r1 / prime256v1) or P-384 (secp384r1) curve. RSA keys are not accepted; the supported cipher suites all use ECDHE-ECDSA.

### Cipher suite and curve requirements

- **Duet 3 Mini 5+ (SAME5x):** Only the `TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256` cipher suite is supported. The server certificate must use an EC P-256 key. P-384 keys and AES-256 cipher suites are not supported.
- **Duet 3 MB6HC / MB6XD (SAME70):** Both `TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256` and `TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384` are supported. P-256 and P-384 keys are both accepted.
- **WiFi module (ESP32-family):** ECDHE-ECDSA cipher suites with AES-128 or AES-256 GCM (SHA-256 / SHA-384). Supported curves are P-256 and P-384, both for the cert's key and for the ECDHE key exchange - matching the MB6HC/MB6XD path so the same cert is portable. RSA, DHE_RSA, ECDH_*, ECDHE_RSA key exchanges and curves outside this set are disabled at compile time. The ESP32-S3 has hardware AES, SHA and big-number acceleration for these operations.
- All modern browsers and TLS clients support P-256 and AES-128-GCM-SHA256. P-256 is the universal choice if you need the cert to work everywhere including the Mini 5+.

### TLS client requirements

**On the Ethernet interface** (SAME5x / SAME70), RRF uses reduced TLS record buffers (2 KiB) to fit within the limited RAM of those targets. For reliable operation, the TLS client should support at least one of:

- `record_size_limit` (RFC 8449) - supported by Firefox; not sent by Chromium-based browsers (Chrome, Microsoft Edge)
- `max_fragment_length` (RFC 6066) - the older TLS 1.2 equivalent

Clients that support neither extension may encounter problems with uploads and large data transfers, because the client may send TLS records larger than the server's 2 KiB input buffer. Some command-line tools (e.g. curl with OpenSSL) also lack support.

Optionally, the server can be configured to reject clients that offer neither extension by enabling `MBEDTLS_SSL_REJECT_MISSING_RECORD_SIZE_EXT` in the mbedTLS config. This is currently disabled but subject to change.

**On the WiFi interface** (ESP32-family) this restriction does not apply - the WiFi firmware uses a full-size 16 KiB TLS input buffer, so any standards-compliant client works regardless of whether it supports the `record_size_limit` or `max_fragment_length` extensions.

### Performance

The TLS handshake involves computationally expensive elliptic-curve operations. Expect the first connection to take approximately 80 ms on the Mini 5+ (Cortex-M4 @ 120 MHz) with the PUKCC hardware accelerator. Subsequent connections from the same client may be faster if the session cache is used.

AES-GCM record encryption and decryption uses the hardware AES peripheral and adds negligible overhead to data transfers.
