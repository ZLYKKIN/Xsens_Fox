// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "FoxLog.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/Paths.h"
#include "Misc/DateTime.h"
#include "Misc/ScopeLock.h"
#include "Serialization/Archive.h"
#include "Interfaces/IPluginManager.h"

DEFINE_LOG_CATEGORY(LogFoxStream);

FFoxLog& FFoxLog::Get()
{
	// C++11 guarantees thread-safe initialization of function-local statics.
	static FFoxLog Instance;
	return Instance;
}

FFoxLog::~FFoxLog()
{
	FScopeLock Lock(&Mutex);
	if (Ar)
	{
		Ar->Flush();
		delete Ar;
		Ar = nullptr;
	}
}

double FFoxLog::Uptime() const
{
	return FPlatformTime::Seconds() - StartSeconds;
}

/** Resolve ue_log.log inside the plugin's own directory (fallback: project Saved/Logs). */
static FString FoxLogResolvePath()
{
	FString Dir;
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("LiveLinkMvnPlugin"));
	if (Plugin.IsValid())
	{
		Dir = Plugin->GetBaseDir();
	}
	else
	{
		Dir = FPaths::ProjectLogDir();
	}
	return FPaths::Combine(Dir, TEXT("ue_log.log"));
}

void FFoxLog::WriteRaw_Locked(const FString& Line)
{
	// Caller holds Mutex.
	if (!Ar)
	{
		return;
	}
	FTCHARToUTF8 Utf8(*Line);
	Ar->Serialize(reinterpret_cast<void*>(const_cast<ANSICHAR*>(Utf8.Get())), Utf8.Length());
}

void FFoxLog::WriteLine_Locked(const FString& Tag, const FString& Fields)
{
	// Caller holds Mutex.
	const FString Line = FString::Printf(TEXT("[+%9.3fs] [%s] %s\n"), Uptime(), *Tag, *Fields);
	UE_LOG(LogFoxStream, Log, TEXT("[%s] %s"), *Tag, *Fields);
	WriteRaw_Locked(Line);
	if (Ar)
	{
		Ar->Flush();
	}
}

void FFoxLog::Reopen(const FString& BannerFields)
{
	FScopeLock Lock(&Mutex);

	if (Ar)
	{
		Ar->Flush();
		delete Ar;
		Ar = nullptr;
	}

	Frames = 0;
	Drops = 0;
	StartSeconds = FPlatformTime::Seconds();
	Path = FoxLogResolvePath();

	// FILEWRITE_None (no FILEWRITE_Append) truncates -> a fresh file every streaming session.
	Ar = IFileManager::Get().CreateFileWriter(*Path, FILEWRITE_None);
	if (!Ar)
	{
		UE_LOG(LogFoxStream, Warning, TEXT("FoxLog: could not open %s"), *Path);
		return;
	}

	WriteLine_Locked(TEXT("boot"), FString::Printf(TEXT("log opened at %s"), *Path));
	WriteLine_Locked(TEXT("boot"), FString::Printf(TEXT("%s plugin=LiveLinkMvnPlugin"), *FDateTime::Now().ToString()));
	if (!BannerFields.IsEmpty())
	{
		WriteLine_Locked(TEXT("boot"), BannerFields);
	}
}

void FFoxLog::Log(const FString& Tag, const FString& Fields)
{
	if (!Ar)
	{
		return;
	}
	FScopeLock Lock(&Mutex);
	if (!Ar)
	{
		return;
	}
	WriteLine_Locked(Tag, Fields);
}

void FFoxLog::Block(const FString& Tag, const FString& HeaderFields, const TArray<FString>& Rows)
{
	if (!Ar)
	{
		return;
	}
	FScopeLock Lock(&Mutex);
	if (!Ar)
	{
		return;
	}

	UE_LOG(LogFoxStream, Log, TEXT("[%s] %s (%d rows)"), *Tag, *HeaderFields, Rows.Num());
	WriteRaw_Locked(FString::Printf(TEXT("[+%9.3fs] [%s] %s\n"), Uptime(), *Tag, *HeaderFields));
	for (const FString& Row : Rows)
	{
		// 16-space indent to match the per-segment block style of fox_mocap.log / blender_log.log
		WriteRaw_Locked(FString::Printf(TEXT("                %s\n"), *Row));
	}
	if (Ar)
	{
		Ar->Flush();
	}
}

void FFoxLog::Close(const FString& SummaryFields)
{
	FScopeLock Lock(&Mutex);
	if (!Ar)
	{
		return;
	}
	WriteLine_Locked(TEXT("stop"), SummaryFields);
	Ar->Flush();
	delete Ar;
	Ar = nullptr;
}

FString FFoxLog::Vec(const FVector& V)
{
	return FString::Printf(TEXT("(%.3f,%.3f,%.3f)"), V.X, V.Y, V.Z);
}

FString FFoxLog::Quat(const FQuat& Q)
{
	// Emit as (w,x,y,z) to match the convention used across the system.
	return FString::Printf(TEXT("q(%.4f,%.4f,%.4f,%.4f)"), Q.W, Q.X, Q.Y, Q.Z);
}
