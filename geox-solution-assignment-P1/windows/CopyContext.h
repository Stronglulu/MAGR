#pragma once
class  CopyContextt {
private:
	struct Entry {
		string name;
		Object *data;
	};
	vector<Entry> tagStack;

public:
	Object *getData(string name);
	void pushData(string name, Object *obj);
	void popData();
};

