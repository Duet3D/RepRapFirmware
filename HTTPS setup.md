# HTTPS / TLS Setup for RepRapFirmware

RRF supports TLS-encrypted protocols on boards with an Ethernet interface (e.g. Duet 3 MB6HC, MB6XD, Mini 5+) and on boards with an ESP32-family WiFi module (Duet 3 MB6HC with the optional WiFi module, and the ESP32 variant of the Duet 3 Mini 5+ WiFi, both running WiFi firmware 2.4.0 or later). All three selectable TCP protocols can be secured with TLS:

| Protocol | Default port | Plain command | TLS variant | Default TLS port |
|----------|-------------|--------------|------------|------------------|
| HTTP     | 80          | `M586 P0 S1` | HTTPS      | 443              |
| FTP      | 21          | `M586 P1 S1` | FTPS       | 990              |
| Telnet   | 23          | `M586 P2 S1` | TelnetS    | 992              |

You need to provide a private key and a certificate in PEM format on the SD card before enabling any TLS variant.

> **WiFi support:** TLS on WiFi requires an ESP32-family module **and** WiFi firmware version 2.4.0 or later. The ESP8266-based WiFi module used on Duet 2 WiFi, FMDC and the original Duet 3 Mini 5+ WiFi cannot host a TLS server - its limited heap (~38 KiB free) cannot accommodate an mbedTLS handshake. On the WiFi path, cert/key are uploaded once to the WiFi module's flash and the SD copies are auto-deleted (see [Section 2](#2-install-the-files-on-the-sd-card) below).

> **SBC mode note:** In SBC mode, HTTPS is handled by DuetWebServer (DSF) via ASP.NET Core / Kestrel and has been supported since DSF 3.3. DSF manages its own certificate independently (`/opt/dsf/conf/https.pfx`), which it auto-generates if not present. The `server.key` and `server.crt` files described in this document are only used by the standalone Ethernet interface - they are not relevant in SBC mode.

## Board and interface capabilities

Which key curve and cipher suites a board accepts depends on the interface that terminates TLS - the MCU for Ethernet, the WiFi module firmware for WiFi:

| Board and interface | TLS terminated by | TLS version | Certificate key curve | ECDHE groups | Cipher suites |
|---------------------|-------------------|-------------|-----------------------|--------------|---------------|
| Duet 3 MB6HC / MB6XD, Ethernet | RRF (SAME70) | 1.3 only | P-256 or **P-384** | P-256, P-384, X25519 | `TLS_AES_128_GCM_SHA256`, `TLS_AES_256_GCM_SHA384` |
| Duet 3 Mini 5+, Ethernet | RRF (SAME54) | 1.3 only | **P-256 only** | P-256 only | `TLS_AES_128_GCM_SHA256` |
| Duet 3 MB6HC with Duet 3 WiFi module (ESP32-S3), WiFi firmware 2.4.0 or later | WiFi firmware | 1.2 only | P-256 or **P-384** | P-256, P-384 | `TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256`, `TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384` |
| Duet 3 Mini 5+ WiFi, ESP32 module variant, WiFi firmware 2.4.0 or later | WiFi firmware | 1.2 only | P-256 or **P-384** | P-256, P-384 | as above |
| Duet 3 Mini 5+ WiFi, ESP8266 module variant | - | no TLS | - | - | - |
| Duet 2 WiFi, FMDC (ESP8266) | - | no TLS | - | - | - |
| Duet 2 Ethernet (W5500) | - | no TLS | - | - | - |
| Any board in SBC mode | DuetWebServer (DSF 3.3 or later) | OS defaults (1.2 and 1.3) | own certificate, EC or RSA | OS defaults | OS defaults |

**Summary:** a P-256 certificate works on every interface that supports TLS at all. P-384 works everywhere except the Duet 3 Mini 5+ Ethernet interface, which is P-256 only because the SAME54 build omits SHA-384 and the P-384 curve to save flash and RAM. If you want one certificate that works on all boards, use P-256.

Notes on the table:

- The WiFi firmware is identical across ESP32-family targets (`DuetWiFiModule_32S3.bin` is the one shipped for the Duet 3 WiFi module; the ESP32 and ESP32-C3 builds share the same TLS configuration), so the WiFi capabilities do not depend on which ESP32 variant is fitted. Only the ESP8266 build has no TLS support at all.
- The Ethernet path serves TLS 1.3 exclusively - clients that offer only TLS 1.2 are rejected. The WiFi path is the opposite: ESP-IDF v4.x ships mbedTLS 2.x, which has no TLS 1.3 server. Every current browser and TLS library handles both.
- RSA certificates are not accepted on any standalone interface - all supported cipher suites use ECDSA. None of these limits apply in SBC mode, where Kestrel serves TLS through the host's OpenSSL and the DuetPi helper script generates an RSA-4096 certificate.
- On the Mini 5+ Ethernet interface, browsers that offer an X25519 key share get a HelloRetryRequest to P-256, which costs one extra round trip (~3 ms on a LAN) per handshake.

---

## 1. Generate a private key and self-signed certificate

The examples below generate an **EC P-256** key and a self-signed certificate valid for 10 years, with the board's hostname set as `duet3`. P-256 is accepted by every interface that supports TLS; some boards also accept P-384, see [Board and interface capabilities](#board-and-interface-capabilities) above.

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

To generate a P-384 key instead, on a board that supports it, replace `prime256v1` with `secp384r1`:

```bash
openssl ecparam -name secp384r1 -genkey -noout -out server.key
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
| `failed to create TLS config` on a Duet 3 Mini 5+ with a certificate that works on other boards | The key is P-384; the Mini 5+ Ethernet interface accepts P-256 only - regenerate with `prime256v1` |
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
- The server certificate **must** use an EC key on the P-256 (secp256r1 / prime256v1) or P-384 (secp384r1) curve, subject to the per-board limits in [Board and interface capabilities](#board-and-interface-capabilities). RSA keys are not accepted.
- On the WiFi path, RSA, DHE_RSA, ECDH_\*, ECDHE_RSA key exchanges and all curves outside P-256/P-384 are disabled at compile time. The ESP32-S3 has hardware AES, SHA and big-number acceleration for the remaining operations.
- All modern browsers and TLS clients support P-256 and AES-128-GCM.

### TLS client requirements

**On the Ethernet interface** (SAME5x / SAME70), RRF uses reduced TLS record buffers (2 KiB) to fit within the limited RAM of those targets. For reliable operation, the TLS client should support at least one of:

- `record_size_limit` (RFC 8449) - supported by Firefox; not sent by Chromium-based browsers (Chrome, Microsoft Edge)
- `max_fragment_length` (RFC 6066) - the older TLS 1.2 equivalent

Clients that support neither extension may encounter problems with uploads and large data transfers, because the client may send TLS records larger than the server's 2 KiB input buffer. Some command-line tools (e.g. curl with OpenSSL) also lack support.

Optionally, the server can be configured to reject clients that offer neither extension by enabling `MBEDTLS_SSL_REJECT_MISSING_RECORD_SIZE_EXT` in the mbedTLS config. This is currently disabled but subject to change.

**On the WiFi interface** (ESP32-family) this restriction does not apply - the WiFi firmware uses a full-size 16 KiB TLS input buffer, so any standards-compliant client works regardless of whether it supports the `record_size_limit` or `max_fragment_length` extensions.

### Performance

The TLS handshake involves computationally expensive elliptic-curve operations. Expect the first connection to take approximately 80 ms on the Mini 5+ (Cortex-M4 @ 120 MHz) with the PUKCC hardware accelerator. Subsequent connections from the same client may be faster if the session is resumed - via session tickets on the Mini 5+, via tickets or the server-side session cache on MB6HC / MB6XD.

AES-GCM record encryption and decryption uses the hardware AES peripheral and adds negligible overhead to data transfers.
