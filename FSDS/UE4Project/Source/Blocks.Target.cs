// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class BlocksTarget : TargetRules
{
	public BlocksTarget(TargetInfo Target) : base(Target)
	{
                DefaultBuildSettings = BuildSettingsVersion.V2;
		Type = TargetType.Game;
		ExtraModuleNames.AddRange(new string[] { "Blocks" });

		// Keep Unreal's default PCH behavior for packaged Linux builds. Disabling
		// PCHs here makes engine modules such as Chaos compile without required
		// standard integer declarations on the bundled Linux toolchain.
		//bUseUnityBuild = false;
	}
}
