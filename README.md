This repository contains the source code for two programs:

- `fan/` — the fan firmware
- `remote/` — the remote control firmware

The `fan` code is modified from the BLEPrph example. It supports multiple BLE connections and is currently configured to accept two connections: one from a smartphone and one from the remote. For more details about how the fan program is structured and how to build/run it, see the README inside the `fan` directory (or the program's internal documentation).

The `remote` connects to a specific UUID (the Alert Notification Service UUID) and looks for the notification service on the peer device. If the notification service is not present the remote will not establish a connection.

Note: there is a timing-related issue where the connection may be dropped the first few attempts (typically 3–5 times). After those initial retries the devices usually connect reliably, and a successful connection is established in under 4 seconds.

Troubleshooting tips:

- If the remote repeatedly fails to connect, check that the target device advertises the Alert Notification Service UUID.
- If initial connections are being dropped, retry a few times — the behavior is expected until the timing condition settles.

If you need anything else added to this README (build steps, wiring, or logs), tell me what to include and I will update it.
