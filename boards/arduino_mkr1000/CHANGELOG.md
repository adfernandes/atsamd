# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.6.0](https://github.com/atsamd-rs/atsamd/compare/arduino_mkr1000-0.5.1...arduino_mkr1000-0.6.0) - 2026-06-18

### Dependencies

- Update to HAL v0.16 ([#878](https://github.com/atsamd-rs/atsamd/pull/878))

## [0.5.1](https://github.com/atsamd-rs/atsamd/compare/arduino_mkr1000-0.5.0...arduino_mkr1000-0.5.1) - 2024-10-17

### Refactored

- update path of Cargo config ([#749](https://github.com/atsamd-rs/atsamd/pull/749))

## v0.5.0

- update to `atsamd-hal-0.14` and other latest dependencies (#564)
- remove extraneous `embedded-hal` dependencies from BSPs
- cleanup `cortex_m` dependency
- move `usbd-x` crates used only in examples to `[dev-dependencies]`
- removed unnecessary dependency on `nb` (#510)

---

Changelog tracking started at v0.4.0
