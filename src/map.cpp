//
//  map.cpp
//  particle_filter
//
//  Created by Silvio Hohne on 8/2/18.
//

#include "map.h"


ostream& operator<<(ostream& os, const Map& map) {
    for (auto lm : map.landmark_list) {
        os << "landmarK: [" << lm.id_i << ", " << lm.x_f << ", " << lm.y_f << "]" << endl;
    }
    return os;
}

