### Unreleased

- Add TX continuation (into RX state) for fast tx-rx turnaround time
- Add auto ack functionality
- Add support for 'raw' messages (aka it's up to the user to encode them as valid ieee 802.15.4 frames)

### v0.6.0 (2021-12-14)

- Add support for double-buffering RX ([#134])
- Add support for RSSI and LOS calculations ([#134])
- Add support for sleep and wakeup ([#134])
- Add support for range bias calculations ([#134])
- Add support for external sync modes ([#134])
- Many small fixes and additions ([#134])
- Update dependencies ([#141], [#145])

[#134]: https://github.com/braun-embedded/rust-dw1000/pull/134
[#141]: https://github.com/braun-embedded/rust-dw1000/pull/141
[#145]: https://github.com/braun-embedded/rust-dw1000/pull/145


### v0.5.0 (2021-01-27)

- Update dependencies ([#123])

[#123]: https://github.com/braun-embedded/rust-dw1000/pull/123


<a name="v0.4.0"></a>
### v0.4.0 (2020-03-30)

- Require `embedded-hal` 0.2.3 ([#112])
- Add more radio configuration options ([#115])
- Update dependencies ([#116])
- Fix panic, improve error reporting in ranging ([#119])


[#112]: https://github.com/braun-embedded/rust-dw1000/pull/112
[#115]: https://github.com/braun-embedded/rust-dw1000/pull/115
[#116]: https://github.com/braun-embedded/rust-dw1000/pull/116
[#119]: https://github.com/braun-embedded/rust-dw1000/pull/119

<a name="v0.3.0"></a>
### v0.3.0 (2019-09-25)

- Upgrade to new `OutputPin` ([#90])
- Various minor documentation updates ([#91], [#107])
- Handle frame filtering rejection ([#92])
- Refactor API to avoid borrowing ([#93], [#101])
- Various minor fixes ([#95])
- Complete definition of SYS_STATE register ([#97])
- Make frame filtering configurable ([#98])
- Only require reference when replying to ranging message ([#99])

[#90]: https://github.com/braun-embedded/rust-dw1000/pull/90
[#91]: https://github.com/braun-embedded/rust-dw1000/pull/91
[#92]: https://github.com/braun-embedded/rust-dw1000/pull/92
[#93]: https://github.com/braun-embedded/rust-dw1000/pull/93
[#95]: https://github.com/braun-embedded/rust-dw1000/pull/95
[#97]: https://github.com/braun-embedded/rust-dw1000/pull/97
[#98]: https://github.com/braun-embedded/rust-dw1000/pull/98
[#99]: https://github.com/braun-embedded/rust-dw1000/pull/99
[#101]: https://github.com/braun-embedded/rust-dw1000/pull/101
[#107]: https://github.com/braun-embedded/rust-dw1000/pull/107


<a name="v0.2.0"></a>
### v0.2.0 (2019-04-20)

- Minor documentation fixes ([#79], [#81], [#82])
- Used `serde` with derive feature instead of `serde_derive` ([#84])
- Update dependency on `ieee802154` to version 0.3 ([#85])
- Remove `macros` module ([#86])

[#79]: https://github.com/braun-robotics/rust-dw1000/pull/79
[#81]: https://github.com/braun-robotics/rust-dw1000/pull/81
[#82]: https://github.com/braun-robotics/rust-dw1000/pull/82
[#84]: https://github.com/braun-robotics/rust-dw1000/pull/84
[#85]: https://github.com/braun-robotics/rust-dw1000/pull/85
[#86]: https://github.com/braun-robotics/rust-dw1000/pull/85


<a name="v0.1.0"></a>
### v0.1.0 (2019-02-20)

Initial release
