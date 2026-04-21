# Changelog

All notable changes to this project will be documented in this file.

## [0.6.0] - 2026-04-21

### 🚀 Features

- Adds pickle and deepcopy support to geo objects

### 🐛 Bug Fixes

- Corrects pickle implementation
- Manually sets PyO3 module for pickle support
- Updated stubs

### 🚜 Refactor

- Aligns Geo objects with py_new pattern

### 🧪 Testing

- Verifies Geo object pickle/deepcopy support
- Adds explicit tests for python pickling

## [0.5.1] - 2026-04-10

### ⚙️ Miscellaneous Tasks

- Updated bevy to 18
- *(release)* Prep for release v0.5.1

## [0.5.0] - 2026-04-02

### 🚀 Features

- Added more utility api functions to geo objects

### 🐛 Bug Fixes

- Updated stubs
- Added either vec or tuple in geopos constructor
- Added to owned stub export derive

### ⚙️ Miscellaneous Tasks

- Locked
- *(release)* Prep for release v0.5.0

## [0.4.1] - 2026-03-17

### 🚀 Features

- Added pydantic schema support to geo objects

### 🐛 Bug Fixes

- Put all pydantic utility functions behind a cfg guarded mod
- Minor typing fix

### ⚙️ Miscellaneous Tasks

- Formatted
- *(release)* Prep for release v0.4.1

## [0.4.0] - 2026-03-06

### 🚀 Features

- Added setter for velocity.ecef_uvw
- Added option to set values in geo references from ref objects
- Added serde methods to geo objects
- Added dict dumping and loading to geo objects
- Added look_to function to orientation
- Added pydantic hooks for geo objects

### 🐛 Bug Fixes

- Put serde behind pyo3 gate
- Fixed unit tests
- Squashed warning from 3.14t
- Fixed stubs
- Fixed issues when building without pydantic-serde feature

### 📚 Documentation

- Updated readme with pydantic integration

### ⚙️ Miscellaneous Tasks

- Prep for release v0.4.0
- Ruff formatted
- Bumped pyo3 stub gen to 19
- Updated deps
- Ruff formatted
- Ruff formatted/fixed
- *(release)* Prep for release v0.4.0

## [0.3.0] - 2026-02-17

### 🚀 Features

- Moved all functions to use impl traits instead of concrete types
- Moved IntoDVec3 trait to return owned vector for greater ergonomics
- Added ref impls for conversion traits
- Added geo objects
- Integrated with pyglam
- Exporting vecs to fix crate boundary issues
- Added getters/setters in geopos
- Added geo orientation and geo velocity
- Added ops for velocity and orientation
- Added EitherLLAOrGeoPos type for more ergonomic constructors
- Added more convenience methods for pos/rot
- Added euler rotation sequences to orientation with unit tests
- Added mach number lookup table
- Added separate ellipsoidal distance function to geopos
- Added optional py_bevy support
- Added bevy only trait
- Added pyo3 feature to block compiling pyo3 at all
- Working compiler features to turn off pyo3 completely from map3d
- Fixed stub generation

### 🐛 Bug Fixes

- Pub export of traits
- Fixed unit test floating point failure
- Minor fix in geo example
- Implemented lerping in speed of sound lookup
- Updated pyglam pointer
- Fixed pyglam pointer
- Updated to work with latest py_bevy
- Working integration with py_bevy macros
- Minor get/set add to velocity
- Updated to work with latest simple-py-bevy
- Added serde and bevy derives
- Fixed bug where #[pyo3] dummy attribute was not passing the token stream through
- Fixed minor bug in pyo3 dummy derive
- Minor stubs fix
- Added pyo3 feature guards to functions that reference pyo3
- Updated paths to pyglam and simple-py-bevy

### 🚜 Refactor

- Reorganized transforms into director
- Reexporting pyglam objects in map3d

### 📚 Documentation

- Updated cargo tomls to be able to upload to crates.io
- Updated readme with crates.io badge
- Minor docstring fixes
- Added docstrings and minor comments
- Added map3d lib docstring
- Minor docstring fixes
- Added simple example and updated readme
- Minor docstring update in mach
- Added docstrings to all geo objects
- Minor docstring update

### 🧪 Testing

- Fixed unit tests
- Added serde tests for geopos

### ⚙️ Miscellaneous Tasks

- Ruff formatted and checked
- Ruff formatted
- Updated deps
- Minor refactor of toml
- Bumped pyglam in lock
- Updated pyglam pointer
- Fixed cargo warning
- Prep for release v0.3.0
- Updated version

## [0.2.3] - 2026-01-12

### 🚀 Features

- Added lla to/from relative coords and default parameters for vincenty functions

### ⚙️ Miscellaneous Tasks

- *(release)* Prep for release v0.2.3

## [0.2.2] - 2026-01-09

### 📚 Documentation

- Added readme to toml

## [0.2.1] - 2026-01-09

### 📚 Documentation

- Minor readme and toml updates

### ⚙️ Miscellaneous Tasks

- *(release)* Prep for release v0.2.1

## [0.2.0] - 2026-01-08

### 🚀 Features

- Add vincenty direct function
- Add DomainError for returning input validation errors
- Add wrap_to_pi function
- Add vincenty inverse function
- Add python bindings for vincenty direct and inverse
- Added stub gen support
- Updated benches
- *(breaking)* Updated all python function apis to not use tuples, matching pymap3d
- Added non uvw transforms for aer, ned, enu

### 🐛 Bug Fixes

- Fixed dagger module pointer
- Updated rust dag function pointer
- Added code checking jobs
- *(api)* Explicitly renamed all relative transformation functions to ecef_uvw to show non absolute positions
- More explicit python interface variable names and docstrings

### 🚜 Refactor

- Reorganized repo to be in line with rust-py projects
- More code organization
- Removed old build scripts
- Separated single file into smaller files

### 📚 Documentation

- More updates to readme
- Added matlab link
- Fixed missing namespace from readme python example
- Add doc string to pymap3d for vicenty functions
- Add function doc string to map3d functions
- Add doc strings to python functions
- Added more docstrings
- Minor python docstring changes
- Minor toml fix
- Minor readme badge fix

### ⚡ Performance

- Change wrap to pi logic to improve runtime

### 🧪 Testing

- Add tests for vincenty direct
- Change vicenty direct test to use relative eq check
- Adding single python-based test for now to be kosher with pytest

### ⚙️ Miscellaneous Tasks

- Added missing meta files
- Added missing stub_gen file
- Removed cargo toml
- Updated version file and ruff fixed
- *(ci)* Added dagger-based ci
- *(ci)* Moved ci jobs out of here into dagger repo
- *(ci)* Removed ci in favor of external dagger project
- *(release)* Prep for release v0.2.0

## [0.1.4] - 2025-03-21

### 🐛 Bug Fixes

- Moved require glam by *
- Fixed rustmap3d:glam dep

## [0.1.3] - 2025-03-20

### 🐛 Bug Fixes

- Moved to using uv in docker container
- Fixed bug in publish job

### ⚙️ Miscellaneous Tasks

- Updated dependency versions

## [0.1.0] - 2024-06-03

### 💼 Other

- Trying to fix unresolved maturin command

<!-- generated by git-cliff -->
