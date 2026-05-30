// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.
//
// FoxLog: persistent, thread-safe logger for the full MVN -> Live Link streaming pipeline.
//
// Writes ue_log.log into the plugin's own directory in the SAME tagged text format as the
// host application's fox_mocap.log, so the whole system (fox_mocap.log + ue_log.log +
// blender_log.log) can be analysed the same way:
//
//     [+   12.345s] [tag] key=value key=value ...
//
// The file is recreated (truncated) on every stream start via Reopen(). Every received frame
// is fully detailed: the parsed header, every raw segment, the MVN->Unreal converted transform
// per bone, and the final transforms pushed to Live Link / written onto the displayed skeleton.
//
// All calls are best-effort and never throw: when the file is not open they are no-ops, so a
// logging failure can never interrupt the mocap stream. Every line is also mirrored to the UE
// Output Log under the LogFoxStream category.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"

class FArchive;

DECLARE_LOG_CATEGORY_EXTERN(LogFoxStream, Log, All);

class FFoxLog
{
public:
	/** Process-wide singleton. */
	static FFoxLog& Get();

	/** Truncate/recreate ue_log.log in the plugin dir and write the boot banner. */
	void Reopen(const FString& BannerFields);

	/** Write a single tagged line: "[+ t] [Tag] Fields". No-op if not open. */
	void Log(const FString& Tag, const FString& Fields);

	/** Write a header line followed by indented detail rows (full per-frame dumps). */
	void Block(const FString& Tag, const FString& HeaderFields, const TArray<FString>& Rows);

	/** Write the [stop] session summary and close the file. */
	void Close(const FString& SummaryFields);

	/** True while the file is open. Use to skip building expensive per-frame dumps. */
	bool IsOpen() const { return Ar != nullptr; }

	/** Per-session counters reported in the [stop] summary. */
	int64 Frames = 0;
	int64 Drops = 0;

	/** Formatting helpers matching the rest of the system (fox_mocap.log / blender_log.log). */
	static FString Vec(const FVector& V);
	static FString Quat(const FQuat& Q);

private:
	FFoxLog() = default;
	~FFoxLog();

	/** Append one line. MUST be called while holding Mutex. */
	void WriteRaw_Locked(const FString& Line);
	void WriteLine_Locked(const FString& Tag, const FString& Fields);
	double Uptime() const;

	FArchive* Ar = nullptr;
	double StartSeconds = 0.0;
	FString Path;
	FCriticalSection Mutex;
};
