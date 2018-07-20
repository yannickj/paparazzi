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
 * Flight_plan module for parsing XML config files
 *)

module OT = Ocaml_tools

type t = {
  filename: string;
  settings: Settings.Dl_setting.t list;
  modules: Module.config list;
  xml: Xml.xml;
}

let from_xml = function
  | Xml.Element ("flight_plan", _, children) as xml ->
    let settings = List.fold_left (fun s el ->
        if Xml.tag el = "variables" then
          s @ List.fold_left (fun s e -> 
              if Xml.tag el = "variable" then
                let test = fun attrib ->
                  match ExtXml.attrib_opt e attrib with
                  | Some _ -> true | None -> false in
                let get_opt = fun attrib -> ExtXml.attrib_opt e attrib in
                if test "min" && test "max" && test "step" then
                  [{ Settings.Dl_setting.var = Xml.attrib e "var";
                     shortname = get_opt "shortname";
                     handler = None;
                     header = None;
                     xml = Xml.Element ("dl_setting", Xml.attribs e, []);
                   }] @ s
                else s
              else s
            ) [] (Xml.children el)
        else
          s
      ) [] children in
    let modules = List.fold_left (fun m el ->
        if Xml.tag el = "modules" then
          m @ List.map Module.config_from_xml (Xml.children el)
        else m
      ) [] children in
    { filename = ""; settings; modules; xml }
  | _ -> failwith "Flight_plan.from_xml: unreachable"

let from_file = fun filename ->
  let fp = from_xml (Xml.parse_file filename) in
  { fp with filename }
