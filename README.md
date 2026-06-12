## What is PlaceNet

PlaceNet is a networking infrastructure built on top of a Proof of Presence (PoP) protocol for creating secure Peer to Peer (P2P) connections with other services within a range of about 1-2km. This is achieved by broadcasting rotating encryption keys over Long Range Radio (LoRa) for both discovery and authentication. Upon verifying that another node is within broadcast distance (~600m - ~2km) using PoP, the nodes will establish a P2P connection via WireGuard with the help of a Coordination server accessible over the open internet. Once connected, PlaceNet nodes are able to share content between themselves as if they were on the same network.

## What is a PlaceNet Beacon?

Beacons are the primary access point for a PlaceNet. They are responsible for broadcasting all information needed to both discover and connect to a local PlaceNet server as well as transmit the final HMAC needed to fully authenticate that a connecting node is within broadcasting distance.

Unlike similar projects like Meshtastic or MeshCore, Beacons do not send content over LoRa minimizing network congestion and drastically increasing the amount of content that can be served over a network.

## Current state of the project

The project is currently in early Alpha. The only supported device is the LilyGO T-Beam but as the project matures more devices are planed to be included such as the Heltec v3. The project was built with making the firmware as compatible with other devices in mind.   

## Similar Projects

PlaceNet is inspired by both Meshtastic, a mesh networking protocol based around offgrid communication with LoRa - and Tailscale, a P2P VPN designed for connecting devices not available over the open internet. 

[Meshatastic](https://meshtastic.org/docs/introduction/)
[Tailscale](https://tailscale.com/blog/how-tailscale-works)
