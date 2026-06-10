# Third-Party Sources

This directory contains vendored algorithm sources used directly by
`fast_lio_sam_sc_qn2`.

- `nano_gicp`: Nano-GICP registration implementation.
- `quatro`: Quatro coarse registration implementation.
- `scancontext_tro`: Scan Context loop candidate detection implementation.

These are intentionally kept as source vendors, not standalone ROS packages.
The package-level `CMakeLists.txt` builds them as:

- `nano_gicp_vendor`
- `quatro_vendor`
- `scancontext_vendor`

Local changes:

- `quatro/src/matcher.cc` uses the `QUATRO_FLANN_CORES` compile definition
  instead of a fixed FLANN thread count.
