/*
 * (C) Copyright 2005
 * Eric JARRIGE, <jorasse@users.sourceforge.net>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

int build_brec_long_read_req(char *a_brecord, long a_addr);
int build_brec_long_write_req(char *a_brecord, long a_addr, long a_value);
int build_brec_mem_write_req(char *a_brecord, long a_addr, void *a_mem, int a_size);
int build_brec_exe_req(char *a_brecord, long a_addr);
