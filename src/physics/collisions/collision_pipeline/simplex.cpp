#include "simplex.hpp"

Simplex::Simplex(){
    this->m_size = 0;
}

Simplex &Simplex::operator=(std::initializer_list<vec3> list)
{
    for (vec3 point : list){
        m_points[m_size++] = point;
    }
        
    return *this;
}

void Simplex::push_front(vec3 point)
{
    m_points = {point, m_points[0], m_points[1], m_points[2]};
    m_size = std::min(m_size + 1, 4);
}

vec3 &Simplex::operator[](int i) { return m_points[i]; }

size_t Simplex::size() const
{
    return m_size;
}

auto Simplex::begin() const
{
    return m_points.begin();
}
auto Simplex::end() const
{
    return m_points.end() - (4 - m_size);
}
