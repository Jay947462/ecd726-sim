# Jetson hardware overlay

This overlay adds guarded oval autonomy to
[Kenou2005/ECD726-CAR](https://github.com/Kenou2005/ECD726-CAR). Use a checkout
that your GitHub account can access. Its `car_drive` package and steering helper
provide the existing VESC guard, firmware setup, calibration, and Foxglove
tooling. Keep the original source and license notices with that checkout.

The overlay was verified against upstream
[`main` at `85b28cd`](https://github.com/Kenou2005/ECD726-CAR/commit/85b28cd35bc055e7044dd075d5c513e23fb0df95).

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

Use the [track-learning guide](docs/TRACK_LEARNING.md) for a fresh map and
operator-reviewed handoff from the mapping lap to replay.
