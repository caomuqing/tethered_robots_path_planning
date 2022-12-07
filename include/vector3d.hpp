#ifndef VECTOR3D_HPP_
#define VECTOR3D_HPP_

#include <vector>

template <typename T>
class vector3d 
{
public:
    vector3d(size_t d1=0, size_t d2=0, size_t d3=0, T const & t=T()) :
        d1(d1), d2(d2), d3(d3), data(d1*d2*d3, t)
    {}

    T & operator()(size_t i, size_t j, size_t k) 
    {
            return (i<=d1 && j<=d2 && k<=d3) ? data[i*d2*d3 + j*d3 + k] 
                                             : data.at(i*d2*d3 + j*d3 + k);
    }

    T const & operator()(size_t i, size_t j, size_t k) const 
    {
        return data[i*d2*d3 + j*d3 + k];
    }

    void resize(const size_t _d1=0, const size_t _d2=0, const size_t _d3=0)
    {
        data.resize(_d1*_d2*_d3);
        d1=_d1;
        d2=_d2;
        d3=_d3;
    }

    void shrink_to_fit()
    {
        data.shrink_to_fit();
    }

    const size_t length() const
    {
        return data.size();
    }

    const size_t capacity() const
    {
        return data.capacity();
    }

    const size_t x() const
    {
        return d1;
    }

    const size_t y() const
    {
        return d2;
    }

    const size_t z() const
    {
        return d3;
    }


private:
    size_t d1,d2,d3;
    std::vector<T> data;
};

template <typename T>
class vector4d 
{
public:
    vector4d(size_t d0=0, size_t d1=0, size_t d2=0, size_t d3=0, T const & t=T()) :
        d0(d0), d1(d1), d2(d2), d3(d3), data(d0*d1*d2*d3, t)
    {}

    T & operator()(size_t p, size_t i, size_t j, size_t k) 
    {
            return (p<=d0 && i<=d1 && j<=d2 && k<=d3) ? data[p*d1*d2*d3 + i*d2*d3 + j*d3 + k] 
                                             : data.at(p*d1*d2*d3 + i*d2*d3 + j*d3 + k);
    }

    T const & operator()(size_t p, size_t i, size_t j, size_t k) const 
    {
        return data[p*d1*d2*d3 + i*d2*d3 + j*d3 + k];
    }


private:
    size_t d0,d1,d2,d3;
    std::vector<T> data;
};

#endif 