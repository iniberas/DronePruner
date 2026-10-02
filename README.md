# Drone Pruner

```sh
git clone --recursive https://github.com/iniberas/DronePruner.git
```

### 1. Install ArduPilot prerequisites

Follow the official ArduPilot guide for your OS:

- [Linux / Ubuntu](https://ardupilot.org/dev/docs/building-setup-linux.html)
- [macOS](https://ardupilot.org/dev/docs/building-setup-mac.html)
- [Windows (WSL)](https://ardupilot.org/dev/docs/building-setup-windows11.html)


### 2. Install PyTorch

Pick one:

```sh
# CPU only
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# With GPU (CUDA)
pip install torch torchvision
```