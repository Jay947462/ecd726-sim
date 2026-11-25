#!/usr/bin/env bash
set -euo pipefail

if [[ $# != 1 ]]; then
    echo "Usage: bash $0 <existing ECD726-CAR checkout>" >&2
    exit 2
fi

overlay=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
checkout=$(cd -- "$1" && pwd -P)
patch_file="$overlay/guarded-autonomy.patch"

patched_files=(
    src/car_drive/launch/drive.launch.py
    src/car_drive/setup.py
    src/car_drive/package.xml
    tools/start_steering.sh
)
added_files=(
    src/car_drive/car_drive/autonomy_node.py
    src/car_drive/config/autonomy.hardware.yaml
    src/car_drive/test/test_autonomy_preview.py
    docs/OVAL_AUTONOMY.md
)

fail() {
    echo "$*" >&2
    exit 1
}

if git_root=$(git -C "$checkout" rev-parse --show-toplevel 2>/dev/null); then
    [[ "$(cd -- "$git_root" && pwd -P)" == "$checkout" ]] ||
        fail "Use the hardware repository root or a standalone checkout: $checkout"
fi

for path in "${patched_files[@]}" "${added_files[@]}"; do
    parent=$(dirname -- "$path")
    while [[ "$parent" != . ]]; do
        [[ -d "$checkout/$parent" && ! -L "$checkout/$parent" ]] ||
            fail "Expected an existing checkout directory: $checkout/$parent"
        parent=$(dirname -- "$parent")
    done
    [[ ! -L "$checkout/$path" ]] || fail "Refusing a symbolic link: $checkout/$path"
done

for path in "${patched_files[@]}"; do
    [[ -f "$checkout/$path" ]] || fail "Required hardware file is missing: $checkout/$path"
done

for path in "${added_files[@]}"; do
    [[ -f "$overlay/$path" ]] || fail "Published overlay file is missing: $overlay/$path"
    if [[ -e "$checkout/$path" ]]; then
        [[ -f "$checkout/$path" ]] && cmp -s -- "$overlay/$path" "$checkout/$path" ||
            fail "Existing file differs from the overlay: $checkout/$path"
    fi
done

git_apply() {
    git -C "$checkout" -c apply.ignoreWhitespace=no apply --whitespace=error "$@" -- "$patch_file"
}

if git_apply --reverse --check 2>/dev/null; then
    echo 'Guarded autonomy patch is already applied.'
elif git_apply --check; then
    git_apply
else
    fail 'The hardware checkout differs from the supported baseline. Review the patch against your version before applying it.'
fi

for path in "${added_files[@]}"; do
    if [[ ! -e "$checkout/$path" ]]; then
        (set -o noclobber; cat -- "$overlay/$path" > "$checkout/$path")
        cmp -s -- "$overlay/$path" "$checkout/$path" ||
            fail "The file changed while applying the overlay: $checkout/$path"
    fi
done

echo "Guarded autonomy overlay is ready in $checkout."
