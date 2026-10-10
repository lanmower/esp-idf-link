# link-esp
I created this unofficial ESP-IDF component for Ableton Link. For more info about Link, see
* https://github.com/ableton/link
* http://ableton.github.io/link

## Installation
* Clone into components folder of a project
* See [example](https://github.com/mathiasbredholt/link-idf-example)

## In this repository
`link/` is a **patched fork of Ableton Link committed directly into this repo** as
plain files -- it is not a submodule, and there is no `.gitmodules` at the repo root.
Do not re-clone it and do not run `git submodule update --init --recursive` against
it: that would silently drop the patch (the multicast relay hook declared at
`link/include/ableton/platforms/asio/Socket.hpp:30` and called on every send), which
still compiles cleanly but stops ESP peer discovery at runtime.
