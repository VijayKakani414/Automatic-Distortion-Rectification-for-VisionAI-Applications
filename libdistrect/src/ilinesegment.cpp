#include <libdistrect.hpp>

namespace distrect
{
	ILineSegment::ILineSegment()
		: a(0.0), b(0.0), invert(false), sx(0.0), sy(0.0), ex(0.0), ey(0.0), segmentNo(0)
	{
	}

	ILineSegment::ILineSegment(const ILineSegment &il)
		: a(il.a), b(il.b), invert(il.invert), sx(il.sx), sy(il.sy), ex(il.ex), ey(il.ey), segmentNo(il.segmentNo)
	{
	}

	ILineSegment::~ILineSegment() {}

} // namespace distrect