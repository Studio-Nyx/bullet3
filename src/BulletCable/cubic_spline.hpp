#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <vector>

using namespace std;

constexpr inline int factorial(int n)
{
    return n <= 1 ? 1 : (n * factorial(n - 1));
}


constexpr inline size_t binomial(size_t n, size_t k) noexcept
{
    return
      (        k> n  )? 0 :          // out of range
      (k==0 || k==n  )? 1 :          // edge
      (k==1 || k==n-1)? n :          // first
      (     k+k < n  )?              // recursive:
      (binomial(n-1,k-1) * n)/k :    //  path to k=1   is faster
      (binomial(n-1,k) * n)/(n-k);   //  path to k=n-1 is faster
}


class CubicCell1D
{
    using Array = std::array<double, 4>;
public:
    explicit CubicCell1D(double x0, double x1, double f0, double f1, double df0, double df1)
      : a(calculate_coefficients(x0, x1-x0, f0, f1, df0, df1))
    {
    }
    ~CubicCell1D() = default;

    const double eval(const double x) const
    {
        return a[0] + (a[1] + (a[2] + a[3]*x)*x)*x;
    }

private:
    const Array alpha_00 {1.0, 0.0, -3.0, +2.0};
    const Array alpha_01 {0.0, 1.0, -2.0, +1.0};
    const Array alpha_10 {0.0, 0.0, +3.0, -2.0};
    const Array alpha_11 {0.0, 0.0, -1.0, +1.0};
    const size_t alpha_size {4};
    const Array a;

    double binomial_power_coefficient(const double y, const int n, const int k) const
    {
		return binomial(n, k)*std::pow(y, n-k);
    }

    void scale_coefficients(std::array<double, 4> &a, const double x0, const double h) const
    {
        using Array = std::array<double, 4>;
        Array dummy {0.0, 0.0, 0.0, 0.0};

        auto i = 0;
        double h_power_i {1.0};
        for (auto &a_i : a)
        {
            auto j = 0;
            for (auto &dummy_j : dummy)
            {
				double value = binomial_power_coefficient(-x0, i, j++) * a_i / h_power_i;
				dummy_j += value;
            }
            h_power_i *= h;
            ++i;
        }

        a.swap(dummy);
    }

    /** Calculates the coefficients for piecewise cubic interpolation cell
        \param x0 x offset
        \param h x scaling factor
        \param f0 value at node 0 (left)
        \param f1 value at node 1 (right)
        \param df0 derivative at node 0 (left)
        \param df1 derivative at node 1 (right
    */
    const Array calculate_coefficients(const double x0, const double h, const double f0, const double f1, const double df0, const double df1) const
    {
        Array coefficients {0.0, 0.0, 0.0, 0.0};
        for (size_t i = 0; i < alpha_size; ++i)
        {
            // note the scaling of df0 and df1, which arises due to differentiation with
            // respect to x (which is scaled by h)
            coefficients[i] += f0*alpha_00[i] + f1*alpha_10[i] + df0*h*alpha_01[i] + df1*h*alpha_11[i];
        }
        scale_coefficients(coefficients, x0, h);
        return coefficients;
    }
};


class BaseSpline
{
private:
	const vector<btScalar> x;
    const size_t index_front = 0;
    const size_t index_back;
    const double x_front;
    const double x_back;
    const double x_delta;
    std::vector<CubicCell1D> splines;
    const size_t cell_index(const double xi) const
    {
        return
        (xi < x_back) ?
            ((xi < x_front) ?
                index_front :
                (size_t)((xi-x_front)/x_delta)) :
            index_back;
    }
    const size_t sort_index(const double xi) const
    {
        if (xi < x_front)
        {
            return index_front;
        }
        if (xi >= x_back)
        {
            return index_back;
        }
        return std::distance(x.begin(), std::upper_bound(x.begin(), x.end(), xi)) - 1;
    }
    const size_t get_index(const double xi) const
    {
        return sort_index(xi);
    }

public:
	BaseSpline(const vector<btScalar> &_x, const vector<btScalar> &_y)
      : x(_x),
        index_back(x.size()-2),
        x_front(x[index_front]),
        x_back(x[x.size()-1]),
        x_delta((x_back-x_front)/(x.size()-1))
    {
        assert(_x.size() == _y.size());
    }
    virtual ~BaseSpline() { }
	virtual const vector<btScalar> calc_slopes(const vector<btScalar> &x, const vector<btScalar> &y) const = 0;
	void build(const vector<btScalar> &x, const vector<btScalar> &y)
    {
		const vector<btScalar> slopes = calc_slopes(x, y);
        splines.reserve(x.size()-1);
        for (int i = 0; i < x.size()-1; ++i)
        {
            splines.push_back(CubicCell1D(x[i], x[i+1], y[i], y[i+1], slopes[i], slopes[i+1]));
        }
    }
    double eval(const double xi) const
    {
		return (splines[get_index(xi+1)].eval(xi+1));
    };
	vector<btScalar> evaln(const vector<btScalar> &xi) const
    {
        auto xi_iter = xi.begin();
		vector<btScalar> yi(xi.size());
        for (auto &yi_i : yi)
        {
            yi_i = eval(*xi_iter++);
        }
        return yi;
    }
};


class MonotonicSpline1D : public BaseSpline
{
public:
	MonotonicSpline1D(const vector<btScalar> &x, const vector<btScalar> &y)
      : BaseSpline(x, y)

    {
        this->build(x, y);
    }
    ~MonotonicSpline1D() { }
	const vector<btScalar> calc_slopes(const vector<btScalar> &x, const vector<btScalar> &y) const override
    {
        // See https://en.wikipedia.org/wiki/Monotone_cubic_interpolation
        auto N = x.size();

        vector<btScalar> secants(N - 1);
		vector<btScalar> tangents(N);

        for (auto k = 0; k < N-1; ++k)
        {
            secants[k] = (y[k+1] - y[k]) / (x[k+1] - x[k]);
        }

        tangents[0] = secants[0];
        for (auto k = 1; k < N-1; ++k)
        {
            tangents[k] = 0.5*(secants[k-1] + secants[k]);
        }
        tangents[N-1] = secants[N-2];

        for (auto k = 0; k < N-1; ++k)
        {
            if (secants[k] == 0.0)
            {
                tangents[k] = 0.0;
                tangents[k+1] = 0.0;
            } else {
                double alpha = tangents[k] / secants[k];
                double beta = tangents[k + 1] / secants[k];
                double h = std::hypot(alpha, beta);
                if (h > 3.0)
                {
                    tangents[k] = 3.0/h*alpha*secants[k];
                    tangents[k+1] = 3.0/h*beta*secants[k];
                }
            }
        }
        return tangents;
    }
};


