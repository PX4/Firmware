#pragma once

#define MODULE_NAME "navigator"

#include <navigator/navigator.h>


class FakeNavigator : public Navigator
{
public:
	FakeNavigator() :
		Navigator() {};

	virtual ~FakeNavigator() {};

private:
};
