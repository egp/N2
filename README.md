<<<<<<< HEAD
# n2

Arduino project developed with Visual Studio Code.

## Requirements

- [Arduino CLI](https://arduino.github.io/arduino-cli/) or [Arduino IDE 2.x](https://www.arduino.cc/en/software)
- [VS Code](https://code.visualstudio.com/) with the [Arduino extension](https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino)

## Hardware

<!-- List your board and any connected components here -->

| Component | Description |
|-----------|-------------|
| Board     |             |

## Getting Started

1. Clone the repo:
   ```sh
   git clone <repo-url>
   cd n2
   ```

2. Open in VS Code:
   ```sh
   code .
   ```

3. Select your board and port via the Arduino extension status bar, then upload.

## Project Structure

```
n2/
├── n2.ino          # Main sketch
└── README.md
```

## License

MIT — see [LICENSE](LICENSE).
=======
# N2

Arduino-based control software for the N2 project.

Current scope includes:

- tower valve control
- O2 handling and display integration
- host-side testable timed state-machine components

## Status

Early development.

## Repository goals

- keep control logic testable on the host side
- minimize blocking behavior in production code
- separate reusable timed state logic from hardware-specific handlers

## License

MIT
>>>>>>> abea67b (chore: initialize N2 repository)
