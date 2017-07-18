(*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *)

(**
 * Autopilot module for parsing XML config files
 *)

type t = {
  modules: Module.t list;
  xml: Xml.xml;
}

from_xml = function
  | Xml.Element ("autopilot", _, children) as xml ->
      let modules = List.fold_left (fun m el ->
        if Xml.tag el = "modules" then
          m @ List.map (fun x -> (* TODO extract module *) ) (Xml.children el)
        else
          m
      ) [] children in
      { modules; xml }
  | _ -> failwith "Autopilot.from_xml: unreachable"

