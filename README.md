# Arduino Fake CAN frames sender - Mazda MX-5 ND

## Description

This Arduino program is a modification of [Timurrrr's "fake car"](https://github.com/timurrrr/RaceChronoDiyBleDevice#testing-with-a-fake-car) code to fake a Mazda MX-5 ND. Please see Timurrrr's instructions how to assemble and use the sender.

This code was created as part of the [KnurDash](https://github.com/chrumck/KnurDash) project.

---

## Serial Console Commands

Apart from functionality as described by Timurrrr. Serial commands were added to the code. Sending `stop` in serial console will stop sending frames, `start` will start, `faster` will reduce the send interval by half, `slower` will double it.
