# Ringfour Platform integration note

The sibling `../platform` project is developing Logbook (milestone 08). Its
initial event ingestion work concerns the Platform API and daemon operation
records. Mission-authored Logbook is a separately installed Platform integration package;
it is not part of the standalone SDK.

`ringfour-sdk` owns the Python client and its documented Agent protocol. If a
future Platform slice needs an SDK integration, verify the existing API in this
repository and pin the tested SDK and Agent versions; do not describe a proposed
Ringfour Platform API as an existing SDK method.

See `../platform/docs/logbook-contract.md` and
`../platform/docs/evidence/08.md` for the current contract and test status.

Mission-authored entries use the separately installed `ringfour-platform-runtime`
package and `from ringfour_runtime import logbook`. The integration is distributed separately from the SDK. The client writes an author-provided
message and structured fields to a private execution-scoped Ringfour runtime;
it is distinct from `robot.audit`, which reads Agent audit records. The Platform contract defines its execution credential and local endpoint;
the local integration tests cover the end-to-end author-entry flow. No Agent
method is implied by this client.

The standalone open-source SDK must work without the Platform integration
package or any Platform service or configuration. The optional package lives in
`../platform/packages/python-runtime` and is installed only where needed.
