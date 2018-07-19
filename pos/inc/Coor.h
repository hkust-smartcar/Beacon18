
#ifndef COOR_H
#define COOR_H

#include <cstdint>

struct Coor
{
	int16_t x;
	int16_t y;
	float theta;

	Coor():x(-1), y(-1), theta(90.0)
	{
	}

	Coor(const int16_t& X, const int16_t& Y, const float& theta=0.0):x(X), y(Y),theta(theta)
	{
	}

	Coor(const Coor& c):x(c.x), y(c.y), theta(c.theta)
	{
	}

	bool operator==(const Coor& r) const
	{
		if(x == r.x && y == r.y)
			return true;
		else
			return false;
	}

	bool operator!=(const Coor& r) const
	{
		return !this->operator==(r);
	}

	void reset()
	{
		x = -1;
		y = -1;
	}

	Coor& operator=(const Coor& r)
	{
		x = r.x;
		y = r.y;
		return *this;
	}

	void Simplfytheta()
	{
		for(;(int)(this->theta)>=360;)
		{
			this->theta-=360;
		}
		for(;(int)(this->theta)<0;)
		{
			this->theta +=360;
		}
	}

};

#endif
