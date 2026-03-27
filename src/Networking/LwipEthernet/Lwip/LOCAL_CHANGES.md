# Local changes to lwIP (based on STABLE-2.2.1)

This file documents all modifications made to the upstream lwIP source
for use in RepRapFirmware.  Keep it up to date when editing lwIP files.

All source files also carry pervasive eCv annotation changes (`const`,
`noexcept`, `_ecv_array`, etc.) for static analysis.  These are not
listed individually below - they affect most files but do not change
runtime behaviour.


## src/core/mem.c + src/include/lwip/mem.h - runtime heap sizing

Replaced the compile-time constant `MEM_SIZE_ALIGNED` with a runtime
variable `lwip_ram_size_aligned`, initialised to the compiled-in maximum.

Added `mem_set_size(mem_size_t size)` which can be called before
`mem_init()` / `lwip_init()` to use a smaller heap than the compiled-in
maximum.  This allows allocating less RAM when TLS is not needed.

All internal references to `MEM_SIZE_ALIGNED` (allocation limits,
free-list traversal, assertions) changed to use `lwip_ram_size_aligned`.


## src/apps/altcp_tls/altcp_tls_mbedtls.c - mbedTLS 3.x / TLS 1.3 port

The upstream file targets mbedTLS 2.x.  This version has been ported to
mbedTLS 3.6 LTS and extended for TLS 1.3 server operation.

### 1. mbedTLS 3.x API migration

- `#include "mbedtls/ssl_internal.h"` -> `#include "ssl_misc.h"`
- Removed `#include "mbedtls/certs.h"` (dropped in 3.x).
- Added `#define MBEDTLS_ALLOW_PRIVATE_ACCESS` for `out_left` access.
- `mbedtls_pk_parse_key()` calls updated to include the RNG parameters
  required by 3.x (`mbedtls_ctr_drbg_random`, `&ctr_drbg`).
- `session->data.start` -> `session->data.MBEDTLS_PRIVATE(start)`.
- `MBEDTLS_SSL_MAX_CONTENT_LEN` -> `MBEDTLS_SSL_IN_CONTENT_LEN`.
- Debug callback: `stdout` -> `NULL` (no stdout on bare-metal).
- `mbedtls_ssl_conf_dbg`: added `#if defined(MBEDTLS_HAVE_TIME)` guard.
- Session save/load functions guarded with `#if defined(MBEDTLS_SSL_CLI_C)`
  since they are only meaningful for TLS clients.

### 2. PSA crypto initialisation

Added `psa_crypto_init()` call during TLS config setup when
`MBEDTLS_PSA_CRYPTO_C` is defined.  Required by mbedTLS 3.x which uses
PSA crypto internally even without `MBEDTLS_USE_PSA_CRYPTO`.

### 3. TLS 1.3 version enforcement

When `MBEDTLS_SSL_PROTO_TLS1_3` is defined, explicitly set min/max TLS
version to 1.3 via `mbedtls_ssl_conf_min/max_tls_version()`.

### 4. WANT_WRITE handshake deadlock fix (lower_sent / lower_poll)

**Problem:** `mbedtls_ssl_handshake()` was only called from
`altcp_mbedtls_lower_recv_process()`, triggered by incoming data.
If the handshake returned `MBEDTLS_ERR_SSL_WANT_WRITE` (TCP send buffer
full), it was never retried - the client waits for the server's flight,
no incoming data arrives, `lower_recv` never fires.

**Fix:** In `altcp_mbedtls_lower_sent()`, after flushing output, check
if the handshake is incomplete and retry via `lower_recv_process()`.
In `altcp_mbedtls_lower_poll()`, same check - retry handshake if not
done, otherwise handle application data as before.

### 5. Correct max output record payload

Replaced the `MBEDTLS_SSL_MAX_FRAGMENT_LENGTH` code path with
`mbedtls_ssl_get_max_out_record_payload()`, which returns the correct
value for TLS 1.3 (OUT_CONTENT_LEN - 1, because the inner content type
byte consumes one byte of payload).

### 6. Robust close_notify and early SSL buffer release

**close_notify:** Upstream called `altcp_close()` without sending a TLS
close_notify.  Now `mbedtls_ssl_close_notify()` is sent before TCP close,
with retry via `ERR_INPROGRESS` if the alert can't be flushed immediately.
Added re-entrancy guard (remove/reattach callbacks) to prevent `lower_err`
freeing the connection while close is in progress.

**Early buffer release:** SSL context buffers (~5 KB per connection) were
held until TCP fully closed (through FIN/ACK/TIME_WAIT), causing heap
accumulation when connections cycle rapidly.  Now `mbedtls_ssl_free()` is
called and `ALTCP_MBEDTLS_FLAGS_HANDSHAKE_DONE` cleared immediately after
close_notify, before `altcp_close()` starts TCP teardown.
`mbedtls_ssl_free()` is idempotent, so the later call in
`altcp_mbedtls_dealloc()` is a safe no-op.

### 7. Improved error handling in lower_recv and tls_write

- Fatal SSL read errors (e.g. oversized records) now abort the connection
  immediately instead of leaving it in a zombie state.
- `lower_recv` for NULL pbuf (remote FIN): simplified to check
  `HANDSHAKE_DONE` directly instead of requiring `UPPER_CALLED` flag.
- `tls_write`: partial writes return bytes written instead of asserting.
  Connection-reset errors (`ERR_CLSD`, `ERR_RST`, `ERR_ABRT`) are mapped
  to `MBEDTLS_ERR_NET_CONN_RESET`.  `WANT_READ` treated same as
  `WANT_WRITE`.
- `altcp_mbedtls_abort()`: added null-check for `inner_conn`.
- `altcp_mbedtls_sndbuf()`: added null-check for `inner_conn`.

### 8. Enhanced handshake failure diagnostics

When `mbedtls_ssl_handshake()` fails with `MBEDTLS_ERR_SSL_FATAL_ALERT_MESSAGE`,
the debug message now prints the TLS alert type code (e.g. 46 = certificate_unknown)
instead of just the generic mbedtls error code.  When `mbedtls_ssl_read()` fails
with a fatal error, the debug message now includes `in_msglen` and
`MBEDTLS_SSL_IN_CONTENT_LEN` to help diagnose oversized record issues.


## src/core/tcp_in.c + src/core/altcp_tcp.c - deferred backlog accept

Upstream calls `tcp_backlog_accepted()` in `tcp_listen_input()` immediately
when the TCP 3-way handshake completes (SYN_RCVD -> ESTABLISHED), *before*
the application accept callback runs.  This makes the backlog counter
useless: the slot is freed before the application decides whether it has
resources to serve the connection, so LwIP keeps SYN-ACKing new
connections that will be immediately aborted.

**Fix:** Changed `tcp_in.c` to call `tcp_backlog_delayed()` instead,
which keeps the backlog slot occupied.  Moved `tcp_backlog_accepted()`
into `altcp_tcp_accept()`, called only when the upper-layer accept
callback returns `ERR_OK`.  On abort/error the existing cleanup paths
(`tcp_abandon`, `tcp_close_shutdown`) already call
`tcp_backlog_accepted()`.

This ensures the listen socket stops SYN-ACKing once backlog slots are
full.  Excess SYNs are silently dropped and the client retries.


## src/core/inet_chksum.c + src/include/lwip/inet_chksum.h

Function signatures changed: `struct pbuf *p` parameters made `const`
where the pbuf contents are not modified.  This is beyond the eCv
annotations - the `const` is semantically meaningful here for callers
that hold const pbuf pointers.


## src/apps/mdns/mdns.c

Replaced verbose rdata hex dump in debug output with a compact one-line
summary (`Type`, `class`, `rd_length`).  Avoids a 200-byte stack buffer
and snprintf loop that was only used for debug logging.
