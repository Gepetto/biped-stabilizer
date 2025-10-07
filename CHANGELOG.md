# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Releases are available on the [github repository](https://github.com/Gepetto/biped-stabilizer/releases).

## [Unreleased]

## [1.4.0] - 1980-01-01

### added

- CI in nix added.
- Add a new representation of the CoP in Vector2d https://github.com/Gepetto/biped-stabilizer/pull/51
- Deprecate the CoP representation in Vector3d https://github.com/Gepetto/biped-stabilizer/pull/51

### changed

- ⚠️ The unit-tests framework was changed from boots-unittests to doctest
- Upgrade of the CMake minimal version to 3.22.1
- Upgrade of the handling of the jrl-cmakemodules dependency in the CMake
- flake8 & black -> ruff

## [1.3.0] - 2023-02-02

### Changed

- ⚠️ Disable python builds by default
- Fix setter for gains
- Fix package.xml

## [1.2.0] - 2022-09-12

### Changed

- Install headers from third-patry

## [1.1.0] - 2022-09-01

### Changed

- Setup CI & badges
- Add setter for gains
- Remove ROS
- Add stabilize method that take a support polygon as input instead of the set of contacts

## [1.0.0] - 2022-08-22

First implementation of the biped-stabilizer.

[Unreleased]: https://github.com/Gepetto/biped-stabilizer/compare/v1.4.0...HEAD
[1.4.0]: https://github.com/Gepetto/biped-stabilizer/compare/v1.3.0...v1.4.0
[1.3.0]: https://github.com/Gepetto/biped-stabilizer/compare/v1.2.0...v1.3.0
[1.2.0]: https://github.com/Gepetto/biped-stabilizer/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/Gepetto/biped-stabilizer/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/Gepetto/biped-stabilizer/releases/tag/v1.0.0
