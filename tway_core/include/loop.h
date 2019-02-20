
#ifndef TWAY_CORE_INCLUDE_LOOP_H_
#define TWAY_CORE_INCLUDE_LOOP_H_

namespace tway {
	class CLoop 	{
	public:
		virtual void ControlLoop() = 0;
		virtual ~CLoop() { }
	};
}

#endif /* TWAY_CORE_INCLUDE_LOOP_H_ */
