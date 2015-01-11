#include "stdafx.h"
#include "CopyContext.h"


Object *CopyContextt::getData(string name) {
	const size_t n = tagStack.size();
	if (n == 0) return NULL;
	for (size_t i = n - 1; i>0; i--) {
		if (tagStack[i].name == name) return tagStack[i].data;
	}
	if (tagStack[0].name == name) return tagStack[0].data;
	return NULL;
}

void CopyContextt::pushData(string name, Object *obj) {
	Entry e;
	e.name = name;
	e.data = obj;
	tagStack.push_back(e);
}

void CopyContextt::popData() {
	pAssert(tagStack.size() > 0);
	tagStack.pop_back();
}
