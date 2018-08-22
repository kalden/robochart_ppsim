

#ifndef ROBOCALC_OPTIONAL_H_
#define ROBOCALC_OPTIONAL_H_

namespace robochart {

template <typename T>
class Optional {
public:
	Optional(): set(false) {}
	Optional(T t): set(true),v(t) {}

	bool Exists() {
		return set;
	}
	T GetValue() {
		return v;
	}
	void SetValue(T t) {
		set = true;
		v = t;
		printf("### CHANGING VALUE OF OPTIONAL\n");
	}
	~Optional() {}
private:
	T v;
	bool set;
};

}

#endif

