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

#include <body.hh>
#include <rocket.hh>
#include <units.hh>
#include <universe.hh>
#include <world.hh>

#include <GL/glu.h>
#include <GL/glut.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/Window.hpp>

#include <array>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <vector>

using namespace std::numbers;

struct View
{
    double x;
    double y;
    double width;
    double height;
    V3 eye;
    V3 at;
    V3 up;
    double mag;
};

int main(int argc, char** argv)
{
    using namespace consts;

    Universe all(false);
    auto orientation = rot(M1, units::deg(23.44)*Vy);
    auto earth = std::make_shared<World>(m_earth, r_earth, V0, V0, orientation,
                                         units::day(1.0));
    all.add(earth);

    std::vector<std::shared_ptr<Body>> v_body;
    std::vector<double> v0s;
    std::vector<std::array<double, 3>> rgb;
    auto r0{4.0*earth->radius()};
    auto v0{1.0*sqrt(earth->m() * G / r0)};
    auto v1{0.005*v0*(argc > 2 ? std::stoi(argv[2]) : 10)};
    auto m{1e20};
    int N = argc > 1 ? std::stoi(argv[1]) : 200;
    for (int i = 0; i < N; ++i)
    {
        auto vi{v1*rot(Vy, i*2.0*pi/N*Vz)};
        v_body.push_back(std::make_shared<Body>(m, M1, r0*Vx, v0*Vy + vi, M1, V0));
        all.add(v_body.back());
        v0s.push_back(mag(v_body.back()->v_cm()));
    }
    auto [v_min, v_max] = std::minmax_element(v0s.begin(), v0s.end());
    for (int i = 0; i < N; ++i)
    {
        auto v{(v0s[i] - *v_min)/(*v_max - *v_min)};
        glColor3d(v, 1-2*abs(v-0.5), 1.0-v);
        rgb.push_back({std::max(v, 0.2), std::max(1-2*abs(v-0.5), 0.2), 1.0-v});
    }
    auto mag = 1/(8.0*r_earth);

    // create the window
    auto width{800};
    auto height{600};
    sf::RenderWindow window(sf::VideoMode(width, height), "Loft",
                      sf::Style::Default, sf::ContextSettings(24));
    window.setVerticalSyncEnabled(true);
    window.setActive(true);

    glutInit(&argc, argv);
    auto* earth_quad = gluNewQuadric();
    gluQuadricDrawStyle(earth_quad, GLU_FILL);
    gluQuadricNormals(earth_quad, GLU_SMOOTH);
    gluQuadricTexture(earth_quad, GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glClearColor(0.1, 0.1, 0.1, 0.0);
    glMatrixMode(GL_PROJECTION);

    sf::Texture earth_tex;
    if (!earth_tex.loadFromFile("earth-texture.png"))
        exit(2);
    sf::Clock clock;
    bool running = true;
    while (running)
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                running = false;
                break;
            case sf::Event::Resized:
                width = event.size.width;
                height = event.size.height;
                break;
            case sf::Event::KeyPressed:
                if (event.key.code == sf::Keyboard::Escape)
                    running = false;
                break;
            default:
                break;
            }
        }

        auto elapsed = clock.restart().asSeconds();
        all.step(1e4*elapsed);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();

        auto dim{std::min(width, height)*mag};
        glOrtho(-width/dim, width/dim, -height/dim, height/dim, -2/mag, 2*r_earth);

        sf::Texture::bind(&earth_tex);
        auto [axis, angle] = axis_angle(earth->orientation());
        glPushMatrix();
        glColor3d(1.0, 1.0, 1.0);
        glRotated((180.0/pi)*angle, axis.x, axis.y, axis.z);
        gluSphere(earth_quad, earth->radius(), 128, 128);
        glPopMatrix();

        glBegin(GL_POINTS);
        glColor3d(0.8, 0.8, 0.8);
        glVertex3f(r0, 0.0, 0.0);
        auto re{earth->r()};
        glVertex3f(0.0, 0.0, 0.0);
        for (std::size_t i = 0; i < v_body.size(); ++i)
        {
            glColor3d(rgb[i][0], rgb[i][1], rgb[i][2]);
            auto r = v_body[i]->r() - re;
            glVertex3f(r.x, r.y, 0.0);
        }
        glEnd();

        glFlush();
        window.display();
        sf::sleep(sf::milliseconds(20));
    }
    return 0;
}
