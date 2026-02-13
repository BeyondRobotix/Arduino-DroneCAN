# Quick Start Guide

Get up and running with hardware tests in 5 minutes!

## Step 1: Install Dependencies

```bash
cd tests
uv sync
```

## Step 2: Configure Your CAN Interface

Edit `test_config.yaml` - change this line:

```yaml
can_interface:
  interface: "slcan:COM3"  # ← Change COM3 to your port
```

**How to find your port:**
- **Windows**: Device Manager → Ports (COM & LPT) → look for your USB device
- **Linux**: Run `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`

## Step 3: Connect Hardware

1. Flash your Arduino DroneCAN firmware
2. Connect CAN adapter to PC (USB)
3. Connect device to CAN adapter (CANH, CANL, GND)
4. Power on device

## Step 4: Run Tests!

```bash
# Quick verification (30 seconds)
uv run test.py --basic

# Full test suite (1-2 minutes)
uv run test.py

# With HTML report
uv run test.py --html
```

## Step 5: Review Results

You should see:
```
✓ ALL TESTS PASSED
```

If tests fail, see [README.md](README.md) troubleshooting section.

## Common First-Time Issues

### Issue: "Failed to connect to CAN interface"
**Fix:** Check your interface string in test_config.yaml matches your adapter

### Issue: "No heartbeat received"
**Fix:**
1. Check device is powered and firmware is running
2. Verify CAN wiring (CANH, CANL, GND)
3. Check node ID in config matches firmware (default: 100)

### Issue: "ModuleNotFoundError: No module named 'dronecan'"
**Fix:** Run `uv sync`

## What's Being Tested?

- ✅ Node heartbeat (device is alive)
- ✅ Message transmission (BatteryInfo @ 10Hz)
- ✅ Message timing accuracy
- ✅ Parameter read/write
- ✅ Message reception handling

## Next Steps

- Read [README.md](README.md) for detailed documentation
- Customize tests for your specific device
- Add tests for your custom messages
- Integrate into your development workflow

Happy testing! 🚀
