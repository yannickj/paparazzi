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
 * Periodic telemetry module for parsing XML config files
 *)

type msg_period = MsgPeriod of float | MsgFreq of float

module Message = struct

  type t = {
    name: string;
    period: msg_period;
    phase: int option;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("message", _, []) as xml ->
      { name = Xml.attrib xml "name";
        period = begin
          match ExtXml.attrib_opt_float xml "period",
                ExtXml.attrib_opt_float xml "freq" with
          | Some t, None -> MsgPeriod t
          | None, Some f -> MsgFreq f
          | Some _, Some _ -> failwith "Telemetry.Message.from_xml: either specify 'period' or 'freq' attribute, not both"
          | None, None -> failwith "Telemetry.Message.from_xml: specify 'period' or 'freq' attribute"
        end;
        phase = ExtXml.attrib_opt_int xml "phase";
        xml
      }
    | _ -> failwith "Telemetry.Message.from_xml: unreachable"

end

module Mode = struct

  type t = {
    name: string;
    key_press: string option;
    messages: Message.t list;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("mode", _, messages) as xml ->
      { name = Xml.attrib xml "name";
        key_press = ExtXml.attrib_opt xml "key_press";
        messages = List.map Message.from_xml messages;
        xml }
    | _ -> failwith "Telemetry.Mode.from_xml: unreachable"

end

module Process = struct

  type t = {
    name: string;
    proc_type: string option;
    modes: Mode.t list;
    xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("process", attribs, modes) as xml ->
      { name = Xml.attrib xml "name";
        proc_type = ExtXml.attrib_opt xml "type";
        modes = List.map Mode.from_xml modes;
        xml }
    | _ -> failwith "Telemetry.Process.from_xml: unreachable"

end

type t = {
  filename: string;
  processes: Process.t list;
  xml: Xml.xml
}

let from_xml = function
  | Xml.Element ("telemetry", [], processes) as xml ->
    { filename = "";
      processes = List.map Process.from_xml processes;
      xml
    }
  | _ -> failwith "Telemetry.from_xml: unreachable"

let from_file = fun filename ->
  let t = from_xml (Xml.parse_file filename) in
  { t with filename }
