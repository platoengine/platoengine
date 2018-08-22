// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

/* Lean Axis-Aligned Bounding box class.
 *
 * Boxes can be efficiently checked for overlapping. This benefits efficient spatial searching
 * with large amounts of data. Two boxes overlap if they intersect in space.
 *
 * A single Id slot is allocated. This Id may be local, global, or not set depending on the
 * needs of the application (be careful).
 *
 * Exclusively for 3D (or lower) objects.
 */

namespace PlatoSubproblemLibrary
{
class Point;

class AxisAlignedBoundingBox
{
public:
    AxisAlignedBoundingBox();
    AxisAlignedBoundingBox(float _m_x_min, float _m_x_max, float _m_y_min, float _m_y_max, float _m_z_min, float _m_z_max, int _m_id = -1);
    AxisAlignedBoundingBox(float _x, float _y, float _z, int _id = -1);

    AxisAlignedBoundingBox(const AxisAlignedBoundingBox box1, const AxisAlignedBoundingBox box2);

    bool overlap(const AxisAlignedBoundingBox& other) const;
    bool overlap_within_tolerance(const AxisAlignedBoundingBox& other, const float tolerance) const;

    bool overlap(const Point& other) const;
    bool overlap_within_tolerance(const Point& other, const float tolerance) const;

    bool contains(const AxisAlignedBoundingBox small) const;

    void set(Point* other);
    void grow_in_each_axial_direction(float growth);
    void grow_to_include(Point* other);

    float get_x_min() const { return m_x_min; }
    float get_x_max() const { return m_x_max; }
    float get_y_min() const { return m_y_min; }
    float get_y_max() const { return m_y_max; }
    float get_z_min() const { return m_z_min; }
    float get_z_max() const { return m_z_max; }

    void set_x_min(float _x) { m_x_min = _x; }
    void set_x_max(float _x) { m_x_max = _x; }
    void set_y_min(float _y) { m_y_min = _y; }
    void set_y_max(float _y) { m_y_max = _y; }
    void set_z_min(float _z) { m_z_min = _z; }
    void set_z_max(float _z) { m_z_max = _z; }

    int get_id() const { return m_id; }

    void set_id(int _id) { m_id = _id; }

    float get_x_center() const { return (m_x_min + m_x_max) * 0.5; }
    float get_y_center() const { return (m_y_min + m_y_max) * 0.5; }
    float get_z_center() const { return (m_z_min + m_z_max) * 0.5; }

    float get_x_length() const { return m_x_max - m_x_min; }
    float get_y_length() const { return m_y_max - m_y_min; }
    float get_z_length() const { return m_z_max - m_z_min; }

    void print(const bool should_end_line) const;

    AxisAlignedBoundingBox(const AxisAlignedBoundingBox& source);
    AxisAlignedBoundingBox& operator=(const AxisAlignedBoundingBox& source);

private:
    float m_x_min, m_x_max, m_y_min, m_y_max, m_z_min, m_z_max;
    int m_id;

};

}
