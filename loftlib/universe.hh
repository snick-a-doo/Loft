//  Copyright (C) 2022 Sam Varner
//
//  This file is part of Laft.
//
//  Loft is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#ifndef LOFT_LOFTLIB_UNIVERSE_HH_INCLUDED
#define LOFT_LOFTLIB_UNIVERSE_HH_INCLUDED

#include <memory>
#include <list>

class Body;

class Universe
{
    using Body_ptr = std::shared_ptr<Body>;

public:
    Universe(bool handle_collision);
    void add(Body_ptr bp);
    void step(double time);

    double time() const;

private:
    bool m_handle_collision{true};
    double m_time{0.0};
    std::list<Body_ptr> m_body;
};

#endif // LOFT_LOFTLIB_UNIVERSE_HH_INCLUDED
