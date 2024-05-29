# CAN Bus Documentation + Troubleshooting

## Troubleshooting

- If the CAN network is UP, but no frames are being recieved and/or the programs are freezing, this could be due to there not being a common ground connection between the ODrive controllers and the CAN network. This could happen if the Jetson and Motors are running off different batteries, but the grounds between them are not tied to eachother.

## CAN Bus Limits

The CAN bus has a max data rate of `1000 Kbps`, which limits the number of commands sent per second and the rate at which info is recieved from the ODrives.

- CAN bus frame size: `144 bits/frame`
- Frames per info: `6 frames/info`
- Info publish rate: `50 infos/second /motor`
- Motors per CAN bus: `6 motors`

`144 * 6 * 50 * 6` = **`259.2 Kbps`**

- Frames per command: `1 frames/command /motor`

`144 * 1 * 6` = **`0.864 Kbits/command`**

So, the maximum number of commands per second is
`(1000 - 259.2) / 0.864` \
= **`857.4 commands/second`**