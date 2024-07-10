### DeckTX

#### Description

A dedicated radio transmitter (TX) is conventionally needed to control drones, but any PC (such as the [Steam Deck](https://store.steampowered.com/steamdeck)) can operate as a TX when connected to a TX module, the component that's responsible for communicating with a drone over radio waves. This is plausible using external TX modules (such as the [BetaFPV SuperG Nano](https://betafpv.com/products/superg-nano-transmitter)) which are sold seperately from the radio transmitter, these often contain a USB port for flashing firmware but can be repurposed for communicating with the module itself.

[ExpressLRS](https://www.expresslrs.org/) is the open-source firmware that most modern TX modules run, these are the target of the project and any device configuration is entirely limited to these devices. The process for connecting the module to a PC involves changing the TX/RX pins to the USB ones if available alongside disabling the backpack since it uses the USB RX/TX pins by default, which needs to be done via changing these values within the ExpressLRS's firmware directly and flashing it or if the TX module supports WiFi then using the `{DEVICE_IP}/hardware.html` page. All approaches are covered in more detail [here](https://github.com/kaack/elrs-joystick-control/blob/2b8031a285bde361b8e9e5339518a4fddbdd51d0/README.md#connecting-to-the-elrs-transmitter).

#### Similar Projects

The core approach is identical to [ELRS Joystick Control](https://github.com/kaack/elrs-joystick-control), the primary difference is that DeckTX is a standalone C++ application that has an integrated UI, any external applications aren't needed to utilize it. On the other hand, ELRS Joystick Control hosts a gRPC server and web-app which needs to be connected to via a browser. Another major difference, is that DeckTX can work directly on a Steam Deck running SteamOS, while ELRS Joystick Control can only work on the Steam Deck when running Windows, at the time of writing.

It should be noted that both approaches have different tradeoffs, a browser-based UI isn't as lightweight or performant as a native application but has far more flexibility in terms of UI design and certain rich components such as maps to display GPS data can be trivially added, while being far more difficult to do on the UI framework used by DeckTX.

#### License

DeckTX is licensed at GPL 3.0 as a whole package, since the vast majority of the ecosystem (ExpressLRS, EdgeTX, etc) uses GPL licenses. Certain files are individually licensed at MIT/MPL 2.0 out of preference for a more open license, this is indicated in the SPDX header.