# Jetson hardware overlay

This overlay adds guarded oval autonomy to an existing ECD726-CAR checkout
with the `car_drive` package and steering helper. It uses the checkout's
existing VESC guard, firmware setup, calibration, and Foxglove tooling.
Keep the original source and license notices with that checkout.

```bash
bash hardware_overlay/apply.sh /path/to/ECD726-CAR
```

The installer checks the four integration files before applying the patch.
It accepts an already-patched checkout and preserves locally edited files.
If an existing calibration or guide differs from the supplied copy, keep that
file and review the changes before applying the overlay. The supplied
calibration has zero placeholders and requires measurements from the car.

The [Jetson bring-up guide](docs/OVAL_AUTONOMY.md) describes the build, manual
Foxglove controls, autonomous monitoring, and stop checks.
