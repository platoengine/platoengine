#pragma once

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class DataSequence;

class DataFlow
{
public:
    DataFlow(AbstractAuthority* authority);
    virtual ~DataFlow();

    void set_data_sequence(DataSequence* data_sequence, const bool& should_free);
    DataSequence* get_data_sequence();

    virtual void unpack_data();
    virtual void allocate_data();

protected:
    DataSequence* m_data_sequence;
    AbstractAuthority* m_authority;
private:
    bool m_should_free_data_sequence;

};

}
