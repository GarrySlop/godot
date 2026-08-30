#!/usr/bin/env bash
# A/B the rendering thread model on a connected Quest, using ONE installed APK.
#
# GodotActivity reads the "command_line_params" intent extra, so `--render-thread`
# can be flipped per launch. That keeps the binary, the assets and the shader cache
# identical across the two runs -- the only difference is the thread model.
#
# Runs are INTERLEAVED (safe, separate, safe, separate, ...). Thermal state drifts
# monotonically over a session, so running all of one arm and then all of the other
# confounds the thread model with the headset's temperature.
#
#   ./run_bench.sh com.example.game                 # 3 reps of each, 300s each
#   ./run_bench.sh com.example.game separate 900 1  # one 900s soak of one arm
set -euo pipefail

PKG="${1:?usage: run_bench.sh <package> [safe|separate|both] [duration_sec] [reps] [activity]}"
WHICH="${2:-both}"
DURATION="${3:-300}"
REPS="${4:-3}"
ACTIVITY="${5:-com.godot.game.GodotApp}"
WARMUP=30
OUT_DIR="bench_results/$(date +%Y%m%d-%H%M%S)"

mkdir -p "$OUT_DIR"

pull_csv() {
	local mode="$1" dest="$2"
	local ext="/sdcard/Android/data/${PKG}/files"
	# Export "Access: External" puts user:// on shared storage; otherwise it is the
	# app's internal dir, reachable only via run-as on a debuggable build.
	local name
	name="$(adb shell "ls ${ext} 2>/dev/null | tr -d '\r' | grep '^bench_${mode}_' | tail -1" || true)"
	if [[ -n "${name}" ]]; then
		adb pull "${ext}/${name}" "${dest}" >/dev/null && return 0
	fi
	name="$(adb shell "run-as ${PKG} ls files 2>/dev/null | tr -d '\r' | grep '^bench_${mode}_' | tail -1" || true)"
	if [[ -n "${name}" ]]; then
		adb shell "run-as ${PKG} cat files/${name}" > "${dest}" && return 0
	fi
	echo "  !! no CSV found for ${mode}; check the app's user:// location" >&2
	return 1
}

run_one() {
	local mode="$1" rep="$2"
	local tag="${mode}_${rep}"
	echo "== ${mode} (rep ${rep}/${REPS}) =="
	adb shell am force-stop "${PKG}" >/dev/null 2>&1 || true
	# Stale CSVs would be picked up by the "latest matching name" search below.
	adb shell "run-as ${PKG} rm -f files/bench_${mode}_*.csv" >/dev/null 2>&1 || true
	adb shell "rm -f /sdcard/Android/data/${PKG}/files/bench_${mode}_*.csv" >/dev/null 2>&1 || true
	adb logcat -c >/dev/null 2>&1 || true
	adb shell am start -S -n "${PKG}/${ACTIVITY}" \
		--esa command_line_params "--render-thread,${mode}" >/dev/null

	local wait_s=$((WARMUP + DURATION + 45))
	echo "  running ~${wait_s}s (keep the headset on its stand, display awake)"
	sleep "${wait_s}"

	adb logcat -d -s 'godot:*' > "${OUT_DIR}/logcat_${tag}.txt" 2>/dev/null || true
	if pull_csv "${mode}" "${OUT_DIR}/${tag}.csv"; then
		echo "  -> ${OUT_DIR}/${tag}.csv"
	fi
	adb shell am force-stop "${PKG}" >/dev/null 2>&1 || true
}

case "${WHICH}" in
	both)
		for ((r = 1; r <= REPS; r++)); do
			run_one safe "$r"
			run_one separate "$r"
		done
		;;
	safe | separate)
		for ((r = 1; r <= REPS; r++)); do
			run_one "${WHICH}" "$r"
		done
		;;
	*)
		echo "second arg must be safe, separate or both" >&2
		exit 2
		;;
esac

echo
echo "Analyse with:"
echo "  python3 misc/xr_benchmark/analyze_bench.py ${OUT_DIR}/safe_*.csv -- ${OUT_DIR}/separate_*.csv"
