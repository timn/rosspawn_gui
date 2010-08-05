
/***************************************************************************
 *  rosspawn_gui.h - Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:15:27 2007
 *  Copyright  2007       Daniel Beck
 *             2008-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __TOOLS_PLUGIN_PLUGIN_GUI_PLUGIN_GUI_H_
#define __TOOLS_PLUGIN_PLUGIN_GUI_PLUGIN_GUI_H_

#include <gtkmm.h>
#include <ros/ros.h>

#include <rosspawn/NodeEvent.h>

class RosSpawnGuiWindow : public Gtk::Window
{
 public:
  RosSpawnGuiWindow(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder> &builder);
  virtual ~RosSpawnGuiWindow();

  void set_ros_node(ros::NodeHandle &n);
  void cb_node_event(const rosspawn::NodeEvent::ConstPtr &msg);

  void on_status_toggled(const Glib::ustring& path);

 private:
  class PluginRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    PluginRecord()
      {
	add(index);
	add(name);
	add(description);
	add(loaded);
	add(status);
      }

    Gtk::TreeModelColumn<int> index;           /**< an index */
    Gtk::TreeModelColumn<Glib::ustring> name;  /**< the name of the plugin */
    Gtk::TreeModelColumn<Glib::ustring> description;  /**< description of the plugin */
    Gtk::TreeModelColumn<bool> loaded;         /**< the loaded status of the plugin */
    Gtk::TreeModelColumn<Glib::ustring> status;  /**< Current status */
  };

 private:
  Glib::RefPtr<Gtk::ListStore> m_plugin_list;
  PluginRecord m_plugin_record;

  Gtk::Statusbar   *m_stb_status;
  Gtk::TreeView    *m_trv_plugins;
  Glib::Dispatcher   __redraw_dispatcher;

  ros::NodeHandle    __n;
  ros::ServiceClient __srv_start;
  ros::ServiceClient __srv_stop;
  ros::ServiceClient __srv_pause;
  ros::ServiceClient __srv_cont;
  ros::ServiceClient __srv_lstld;
  ros::ServiceClient __srv_lstav;
  ros::Subscriber    __sub_node_events;
};

#endif
