# Working in OpenVCX

Read README.md and the relevant source before editing. Be concise, preserve
unrelated changes, and keep user instructions ahead of repository conventions.

Keep user-facing instructions in README.md, configuration in
CONFIGURATION.md, and developer details in CONTRIBUTING.md. Avoid adding
audit narratives, duplicate checklists or references to unavailable private logs.
Keep license and source-origin notices.

For C/build changes, run build_DLL_core.ps1 for x86 and x64 with offline tests and
the export check. For docs, check claims, links and whitespace. Do not access
hardware, install providers or actuate a vehicle merely to satisfy an offline test.

Distinguish actual vehicle results from offline tests and estimates. Keep private
captures, VINs, vendor files, firmware, toolchains and binaries out of Git.
Report checks and whether changes were committed or pushed.
