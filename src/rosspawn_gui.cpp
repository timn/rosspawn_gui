
/***************************************************************************
 *  rosspawn_gui.cpp -  Plugin Tool Gui
 *
 *  Created: Thu Nov 09 20:16:23 2007
 *  Copyright  2007       Daniel Beck
 *             2008-2009  Tim Niemueller [www.niemueller.de]
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

#include <rosspawn/NodeAction.h>
#include <rosspawn/ListLoaded.h>
#include <rosspawn/ListAvailable.h>

#include <string>

/** @class RosSpawnGuiWindow "rosspawn_gui.h"
 * Graphical plugin management tool.
 *
 * @author Tim Niemueller
 * @author Daniel Beck
 */

/** Constructor.
 * @param cobject C base object
 * @param ref_xml Glade XML
 */
RosSpawnGuiWindow::RosSpawnGuiWindow(BaseObjectType* cobject,
				     const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::Window(cobject)
{
  builder->get_widget("stb_status", m_stb_status);
  builder->get_widget("trv_plugins", m_trv_plugins);

  m_plugin_list = Gtk::ListStore::create(m_plugin_record);
  m_trv_plugins->set_model(m_plugin_list);
  m_trv_plugins->append_column("#", m_plugin_record.index);
  m_trv_plugins->append_column_editable("Status", m_plugin_record.loaded);
  m_trv_plugins->append_column("Plugin", m_plugin_record.name);
  m_trv_plugins->append_column("Status", m_plugin_record.status);

  __redraw_dispatcher.connect(sigc::mem_fun(*this, &Gtk::Window::queue_draw));

  Gtk::CellRendererToggle* renderer;
  renderer = dynamic_cast<Gtk::CellRendererToggle*>( m_trv_plugins->get_column_cell_renderer(1) );
  renderer->signal_toggled().connect( sigc::mem_fun(*this, &RosSpawnGuiWindow::on_status_toggled));

  m_stb_status->push("Started");
}

/** Destructor. */
RosSpawnGuiWindow::~RosSpawnGuiWindow()
{
  m_stb_status->push("Exiting");
}


void
RosSpawnGuiWindow::cb_node_event(const rosspawn::NodeEvent::ConstPtr &msg)
{
  // Find row

  Gtk::TreeIter iter;
  for ( iter  = m_plugin_list->children().begin();
	iter != m_plugin_list->children().end();
	++iter )
  {
    Glib::ustring n = (*iter)[m_plugin_record.name];
    if ( n == msg->node_name ) {
      switch (msg->event_type) {
      case rosspawn::NodeEvent::NODE_STARTED:
	(*iter)[m_plugin_record.loaded] = true;
	(*iter)[m_plugin_record.status] = "running";
	break;

      case rosspawn::NodeEvent::NODE_DIED:
	(*iter)[m_plugin_record.loaded] = false;
	(*iter)[m_plugin_record.status] = "stopped";
	break;

      case rosspawn::NodeEvent::NODE_PAUSED:
	(*iter)[m_plugin_record.loaded] = true;
	(*iter)[m_plugin_record.status] = "paused";
	break;

      case rosspawn::NodeEvent::NODE_CONTINUED:
	(*iter)[m_plugin_record.loaded] = true;
	(*iter)[m_plugin_record.status] = "continued";
	break;

      case rosspawn::NodeEvent::NODE_SEGFAULT:
	(*iter)[m_plugin_record.loaded] = true;
	(*iter)[m_plugin_record.status] = "segfault";
	break;
      default:
	break;
      }
    }

    break;
  }

  __redraw_dispatcher();
}


/** Signal handler that is called when the loaded checkbox is
 * toggled.
 * @param path the path of the selected row
 */
void
RosSpawnGuiWindow::on_status_toggled(const Glib::ustring& path)
{
  Gtk::TreeModel::Row row = *m_plugin_list->get_iter(path);
  Glib::ustring node_file_name = row[m_plugin_record.name];
  bool loaded = ! row[m_plugin_record.loaded];

  rosspawn::NodeAction na;
  na.request.node_file_name = node_file_name;

  if (loaded) {
    __srv_stop.call(na);
  } else {
    __srv_start.call(na);
  }
}

void
RosSpawnGuiWindow::set_ros_node(ros::NodeHandle &n)
{
  __n = n;
  __srv_start = __n.serviceClient<rosspawn::NodeAction>("/rosspawn/start");
  __srv_stop  = __n.serviceClient<rosspawn::NodeAction>("/rosspawn/stop");
  __srv_pause = __n.serviceClient<rosspawn::NodeAction>("/rosspawn/pause");
  __srv_cont  = __n.serviceClient<rosspawn::NodeAction>("/rosspawn/continue");
  __srv_lstld = __n.serviceClient<rosspawn::ListLoaded>("/rosspawn/list_loaded");
  __srv_lstav = __n.serviceClient<rosspawn::ListAvailable>("/rosspawn/list_available");

  __sub_node_events = __n.subscribe("/rosspawn/node_events", 10,
				    &RosSpawnGuiWindow::cb_node_event, this);

  rosspawn::ListAvailable lstav;
  if (__srv_lstav.call(lstav)) {
    for (std::vector<std::string>::iterator i = lstav.response.bin_files.begin();
	 i != lstav.response.bin_files.end(); ++i) {

      Gtk::TreeModel::Row row = *m_plugin_list->append();
      unsigned int index = m_plugin_list->children().size();
      row[m_plugin_record.index]       = index;
      row[m_plugin_record.name]        = *i;
      row[m_plugin_record.description] = "";
      row[m_plugin_record.loaded]      = false;
      row[m_plugin_record.status]      = "unknown";
    }
  }

  rosspawn::ListLoaded lstld;
  if (__srv_lstld.call(lstld)) {
    for (std::vector<std::string>::iterator i = lstld.response.nodes.begin();
	 i != lstld.response.nodes.end(); ++i) {

      Gtk::TreeIter iter;
      for ( iter  = m_plugin_list->children().begin();
	    iter != m_plugin_list->children().end();
	    ++iter )
      {
	Glib::ustring n = (*iter)[m_plugin_record.name];
	if ( n == *i ) {
	  (*iter)[m_plugin_record.loaded] = true;
	  (*iter)[m_plugin_record.status]     = "running";
	  break;
	}
      }
    }
  }
}
