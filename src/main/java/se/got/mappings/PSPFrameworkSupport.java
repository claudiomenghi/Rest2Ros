/**
 * Copyright (C) 2011-2014 Swinburne University of Technology and University of Gotheborg
 *
 * These file have been developed as a part of the co4robots project.
 * It is a tool
 *
 * These files are based on PSPWizard which was developed at Faculty of Science, Engineering and
 * Technology at Swinburne University of Technology, Australia.
 * The patterns, structured English grammar and mappings are due to
 * Marco Autili, Universita` dell'Aquila
 * Lars Grunske, University of Stuttgart
 * Markus Lumpe, Swinburne University of Technology
 * Patrizio Pelliccione, University of Gothenburg
 * Antony Tang, Swinburne University of Technology
 *
 * Details about the PSP framework can found in
 * "Aligning Qualitative, Real-Time, and Probabilistic
 * Property Specification Patterns Using a Structured
 * English Grammar"
 *
 *
 *
 * PSPWizard is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * PSPWizard is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PSPWizard; see the file COPYING.  If not, write to
 * the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */
package se.got.mappings;

import se.got.constraints.Interval;
import se.got.constraints.ProbabilityBound;
import se.got.constraints.TimeBound;
import se.got.sel.Event;
import se.got.sel.patterns.order.ChainEvents;

public interface PSPFrameworkSupport 
{
    public String cnt( Event aZP );

    public String time( TimeBound aPTimeBound );
    public String lmintime( TimeBound aPTimeBound );
    public String umintime( TimeBound aPTimeBound );
    public String utb( TimeBound aPTimeBound );
    
    // precedence support (interval only)
    public String trigger( Interval aPTimeBound );
    public String gap( Interval aPTimeBound );
    public String elapsed( Interval aPTimeBound );
    public String maxgap( Interval aPTimeBound );
    public String gapNP( int n, ChainEvents Tis, Interval aPTimeBound );
    public String gapPN( int n, ChainEvents Tis, Interval aPTimeBound );
    
    public String tL( TimeBound aPTimeBound );
    public String tU( TimeBound aPTimeBound );

    public String prop( ProbabilityBound aPropBound );
}
