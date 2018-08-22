
#ifndef ROBOCALC_EVENT_H_
#define ROBOCALC_EVENT_H_

#include "Optional.h"
#include <vector>
#include <sstream>
#include <memory>

#define EVENT_DEBUG

namespace robochart {

template<typename ...Args>   //variadic template
class Event {
private:
	std::string channel;
	std::string source;
	std::shared_ptr<std::tuple<Args...>> parameters;
	Optional<std::weak_ptr<Event<Args...>>> other;
	bool accepted;
public:
	Event(std::string c, std::string id, Args ... args) :
			channel(c), source(id), other(), accepted(false), parameters(
					std::make_shared<std::tuple<Args...>>(std::make_tuple(args...))) {
	}
	virtual ~Event() {
	}
	std::shared_ptr<std::tuple<Args...>> GetParameters() {
		return parameters;
	}
	void SetParameter(const uint i, std::string s) {
		std::get<i>(parameters).setValue(s);
	}
	Optional<std::string> GetSource() {
		return source;
	}
	Optional<std::weak_ptr<Event<Args...>>> GetOther() {
		return other;
	}
	void SetOther(std::weak_ptr<Event<Args...>> s) {
		other = Optional<std::weak_ptr<Event<Args...>>>(s);
	}
	void Accept() {
		accepted = true;
	}
	bool IsAccepted() {
		return accepted;
	}

	/* if two events come from different sources and their types are the same, these two events are compatible
	 * e1.value().compare(e2.value()) != 0: if the two strings are different,
	 * which means the two events comes from different components, we say it is compatible
	 * */
	bool Compatible(std::shared_ptr<Event<Args...>> e) {
		Optional<std::string> e1, e2;
		e1 = GetSource();
		e2 = e->GetSource();
		if (e1.Exists() && e2.Exists() && e1.GetValue().compare(e2.GetValue()) != 0) {  //&& getParameters().size() == e->getParameters().size()
#ifdef EVENT_DEBUG
			printf("Event compatible!\n");
#endif
			return true;
		} else {
#ifdef EVENT_DEBUG
			printf("Event not compatible!\n");
#endif
			return false;
		}
	}

	/* set the reference of the matched event for sync/async communication
	 *
	 */
	void Match(std::shared_ptr<Event<Args...>> e) {
		if (!Compatible(e))
			return;
		SetOther(e);
	}

	std::string GetChannel() {
		return channel;
	}
};

}

#endif

