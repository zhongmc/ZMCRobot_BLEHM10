#include "utils.h"


int getCrossPoint(Vector p0, Vector p3, Vector p1, Vector p2, Vector &result)
{
    if (!doesVectorCross(p0, p3, p1, p2))
		return 0;

	double a1 = (p3.y - p0.y) / (p3.x - p0.x);
	double b1 = p3.y - a1 * p3.x;

	if (p1.y == p2.y) {
		result.x = (float) ((p1.y - b1) / a1);
		result.y = p1.y;
	    return 1;
    }


		if (p1.x == p2.x) {
			result.x = p1.x;
			result.y = (float) (a1 * p1.x + b1);
			return 1;
			// return new PointF(p1.x, (float)(a1*p1.x + b1 ));
		}

		double a = (p2.y - p1.y) / (p2.x - p1.x);
		double b = p2.y - a * p2.x;

		result.x = (b1 - b) / (a - a1);
		result.y = a * result.x + b;
		return 1;    


}

bool doesVectorCross(Vector P0, Vector P1, Vector Q0, Vector Q1)
{
		boolean val = (max(P0.x, P1.x) >= min(Q0.x, Q1.x)) && (max(Q0.x, Q1.x) >= min(P0.x, P1.x))
				&& (max(P0.y, P1.y) >= min(Q0.y, Q1.y)) && (max(Q0.y, Q1.y) >= min(P0.y, P1.y));

		if (!val)
			return false;

		// (p0-q0)*(q1-q0)
		double p1xq = CrossProduct(P0.x - Q0.x, P0.y - Q0.y, Q1.x - Q0.x, Q1.y - Q0.y);

		// (p1-q0)*(q1-q0)
		double p2xq = CrossProduct(P1.x - Q0.x, P1.y - Q0.y, Q1.x - Q0.x, Q1.y - Q0.y);
		// (q0-p0)*(p1-p0)
		double q1xp = CrossProduct(Q0.x - P0.x, Q0.y - P0.y, P1.x - P0.x, P1.y - P0.y);

		// (q1-p0)*(p1-p0)
		double q2xp = CrossProduct(Q1.x - P0.x, Q1.y - P0.y, P1.x - P0.x, P1.y - P0.y);

		return (p1xq * p2xq <= 0) && (q1xp * q2xp <= 0);
}

double CrossProduct(double x1, double y1, double x2, double y2)
 {
		return x1 * y2 - x2 * y1;
}

