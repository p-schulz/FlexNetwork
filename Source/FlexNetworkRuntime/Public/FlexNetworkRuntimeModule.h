#pragma once

#include "Modules/ModuleManager.h"

class FFlexNetworkRuntimeModule final : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
