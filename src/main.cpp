
/***************************************************************************
 *  main.cpp - Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:13:45 2007
 *  Copyright  2007  Daniel Beck
 *             2010  Tim Niemueller
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "rosspawn_gui.h"
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosspawn_gui", ros::init_options::NoSigintHandler);
  Gtk::Main kit(argc, argv);
  
  Glib::RefPtr<Gtk::Builder> builder =
    Gtk::Builder::create_from_file(RESDIR"/rosspawn_gui.ui");

  RosSpawnGuiWindow *window = NULL;
  builder->get_widget_derived("wnd_main", window);

  ros::NodeHandle n;
  window->set_ros_node(n);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  kit.run( *window );

  spinner.stop();

  delete window;
  return 0;
}
