

#pragma once

#include "Engine.h"

#include <string>


// Module-global utilities.
template <typename ObjClass>
static ObjClass* LoadObjFromPath(const FName& Path)
{
	if (Path == NAME_None) return NULL;
	return Cast<ObjClass>(StaticLoadObject(ObjClass::StaticClass(), NULL, *Path.ToString()));
}

std::wstring widen(const std::string& s);
