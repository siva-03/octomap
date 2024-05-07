#include <iostream>
#include <cassert>

using namespace std;

extern double resolution = 0.01;
extern double resolution_factor = 1.0 / resolution;
extern unsigned int tree_max_val = 32768;
extern int tree_depth = 16;
/// contains the size of a voxel at level i (0: root node). tree_depth+1 levels (incl. 0)
std::vector<double> sizeLookupTable;

#include <bitset>

void printBinary(uint16_t number)
{
    std::bitset<16> binary(number);
    std::cout << "Binary representation of " << number << " is: " << binary << std::endl;
}

/// Converts from a single coordinate into a discrete key at tree_depth
uint16_t coordToKey(double coordinate)
{
    return ((int)floor(resolution_factor * coordinate)) + tree_max_val;
}

/// Converts from a single coordinate into a discrete key at a given depth
uint16_t coordToKey(double coordinate, unsigned depth)
{
    assert(depth <= tree_depth);
    int keyval = ((int)floor(resolution_factor * coordinate));

    unsigned int diff = tree_depth - depth;
    if (!diff) // same as coordToKey without depth
        return keyval + tree_max_val;
    else // shift right and left => erase last bits. Then add offset.
    {
        // cout << "one: " << ((keyval >> diff) << diff) << endl;
        // cout << "two: " << (1 << (diff - 1)) << endl;
        return ((keyval >> diff) << diff) + (1 << (diff - 1)) + tree_max_val;
    }
}

/**
 * Adjusts a single key value from the lowest level to correspond to a higher depth (by
 * shifting the key value)
 *
 * @param key Input key, at the lowest tree level
 * @param depth Target depth level for the new key
 * @return Key for the new depth level
 */
uint16_t adjustKeyAtDepth(uint16_t key, unsigned int depth)
{
    unsigned int diff = tree_depth - depth;

    if (diff == 0)
        return key;
    else
        return (((key - tree_max_val) >> diff) << diff) + (1 << (diff - 1)) + tree_max_val;
}

/// converts from a discrete key at the lowest tree level into a coordinate
/// corresponding to the key's center
double keyToCoord(uint16_t key)
{
    return (double((int)key - (int)tree_max_val) + 0.5) * resolution;
}

/// converts from a discrete key at a given depth into a coordinate
/// corresponding to the key's center
double keyToCoord(uint16_t key, unsigned depth)
{
    assert(depth <= tree_depth);
    assert(sizeLookupTable[0] == 655.36);
    assert(sizeLookupTable[16] == 0.01);

    // root is centered on 0 = 0.0
    if (depth == 0)
    {
        return 0.0;
    }
    else if (depth == tree_depth)
    {
        return keyToCoord(key);
    }
    else
    {
        return (floor((double(key) - double(tree_max_val)) / double(1 << (tree_depth - depth))) + 0.5) * sizeLookupTable[depth];
    }
}

/**
 * Computes the key of a child node while traversing the octree, given
 * child index and current key
 *
 * @param[in] pos index of child node (0..7)
 * @param[in] center_offset_key constant offset of octree keys
 * @param[in] parent_key current (parent) key
 * @param[out] child_key  computed child key
 */
uint16_t computeChildKey(unsigned int pos, uint16_t center_offset_key,
                         uint16_t parent_key)
{
    uint16_t child_key;
    // x-axis
    if (pos & 1)
        child_key = parent_key + center_offset_key;
    else
        child_key = parent_key - center_offset_key - (center_offset_key ? 0 : 1);
    // // y-axis
    // if (pos & 2)
    //     child_key[1] = parent_key[1] + center_offset_key;
    // else
    //     child_key[1] = parent_key[1] - center_offset_key - (center_offset_key ? 0 : 1);
    // // z-axis
    // if (pos & 4)
    //     child_key[2] = parent_key[2] + center_offset_key;
    // else
    //     child_key[2] = parent_key[2] - center_offset_key - (center_offset_key ? 0 : 1);
    return child_key;
}

/// generate child index (between 0 and 7) from key at given tree depth
int computeChildIdx(uint16_t key, int depth)
{
    // only visualizing for one axis X
    int pos = 0;
    if (key & (1 << depth))
        pos += 1;

    // if (key.k[1] & (1 << depth))
    //     pos += 2;

    // if (key.k[2] & (1 << depth))
    //     pos += 4;

    return pos;
}

/**
 * Generates a unique key for all keys on a certain level of the tree
 *
 * @param level from the bottom (= tree_depth - depth of key)
 * @param key input indexing key (at lowest resolution / level)
 * @return key corresponding to the input key at the given level
 */
uint16_t computeIndexKey(uint16_t level, uint16_t key)
{
    if (level == 0)
        return key;
    else
    {
        uint16_t mask = 65535 << level;
        uint16_t result = key;
        result &= mask;
        return result;
    }
}

int main()
{
    cout << coordToKey(163.84) << endl; // 49152
    cout << coordToKey(200, 3) << endl; // 53248
    // for(int depth = 0; depth <= 16; depth++)
    // {
    //     cout << depth << endl;
    //     cout << coordToKey(163.84, depth) << endl;
    //     cout << endl;
    // }

    // This just changes the tree_depth level key (key at the lowest level) into the right key at a given depth
    // Look at the diagram in the book, more like choosing which key bucket this falls into
    cout << adjustKeyAtDepth(35000, 1) << endl; // 49152

    cout << keyToCoord(32768) << endl; // 0.005 - because it is the center of 0.0 and 0.01

    // init node size lookup table:
    // cout << "Sizelookuptable: voxel size at level i" << endl;
    sizeLookupTable.resize(tree_depth + 1);
    for (unsigned i = 0; i <= tree_depth; ++i)
    {
        sizeLookupTable[i] = resolution * double(1 << (tree_depth - i));
        // cout << i << ": " << sizeLookupTable[i] << endl;
    }

    cout << keyToCoord(32768, 1) << endl; // 163.84
    cout << keyToCoord(6000, 1) << endl;  // -163.84

    uint16_t center_offset_key;
    // key_type center_offset_key = this->tree_max_val >> (depth + 1); // They do this to calc child key 1 deeper
    for (int i = 0; i <= tree_depth; i++)
    {
        center_offset_key = (tree_max_val >> (i));
        cout << "i: " << i << " centoffkey: " << center_offset_key << endl;
        cout << computeChildKey(1, center_offset_key, 16384) << endl;
    }

    // see take 16384 on depth 2 and check its left and right child keys.
    center_offset_key = (tree_max_val >> (1 + 1));                // cur_depth + 1
    cout << "centr_offset_key " << center_offset_key << endl;
    cout << computeChildKey(1, center_offset_key, 16384) << endl; // 24756 - right child key
    cout << computeChildKey(0, center_offset_key, 16384) << endl; // 8192 - lc key

    // // compute child idx
    // printBinary(16384);
    // printBinary(57344);
    // printBinary(24576);

    // given a key, take it to the level u need and see if it falls into 
    // 0 means to left, 1 means to right
    
    // cout << computeChildIdx(16384, 16 - 1) << endl; // 0
    // cout << computeChildIdx(57344, 16 - 2) << endl; // 1
    // cout << computeChildIdx(24576, 16 - 1) << endl; // 0
    // cout << computeChildIdx(24576, 16 - 2) << endl; // 1
    // cout << computeChildIdx(24576, 16-3) << endl; // 1 since non inclusive on right - refer arrow in diagram
    // cout << computeChildIdx(24576, 16-4) << endl; // 0
    // cout << computeChildIdx(65535, 16) << endl; // 0 since on root level everything
    cout << computeChildIdx(26000, 16 - 1) << endl; // 0
    cout << computeChildIdx(26000, 16 - 2) << endl; // 1

    // computeIndexKey - i dont know what this does yet
    // cout << computeIndexKey(16 - 1, 40960) << endl;


}