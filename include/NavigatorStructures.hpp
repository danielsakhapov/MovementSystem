#pragma once

using u32 = uint32_t;
using vec_bool = std::vector<bool>;
using pair_u32x = std::pair<u32, u32>;
using pair_dx = std::pair<double, double>;
using vec_pair_u32x = std::vector<pair_u32x>;
using vec_vec_bool = std::vector<std::vector<bool>>;
using vec_vec_pair_u32x = std::vector<std::vector<pair_u32x>>;

template<typename T>
struct Position
{
	T x;
	T y;
	int theta;
};

using uPosition = Position<u32>;
using dPosition = Position<double>;
using vec_uPosition = std::vector<uPosition>;

struct Engine
{
	int angle;
	int speed;
	int direction;
};
