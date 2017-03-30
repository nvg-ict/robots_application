#ifndef POINT_HPP_
#define POINT_HPP_

/*
 * Copyright (c) 2016 Joost Kraaijeveld. See license.txt for details.
 * For information, bug reports and additions send an e-mail to DistributedRobotWorld@Askesis.nl.
 *
 * Author: jkr
 */

//#include <wx/gdicmn.h>
//
//namespace Widgets
//{
//	/**
//	 * @see http://docs.wxwidgets.org/stable/classwx_point
//	 */
//	typedef wxPoint Point;
//} // namespace Widgets
//using namespace Widgets;

struct Point
{
	Point(int aX, int aY)
	:x(aX),y(aY)
	{
	}
	int x;
	int y;
};

#endif // POINT_HPP_
