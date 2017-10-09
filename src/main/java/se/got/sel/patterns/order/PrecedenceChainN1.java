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
package se.got.sel.patterns.order;

import java.util.ArrayList;

import se.got.constraints.EventConstraint;
import se.got.constraints.ProbabilityBound;
import se.got.constraints.TimeBound;
import se.got.engine.PSPConstants;
import se.got.sel.Event;
import se.got.sel.patterns.Order;

public class PrecedenceChainN1 extends Order
{
    private TimeBound fTimeBoundP;
    private EventConstraint fPConstraint;
 
    public TimeBound getPTimeBound()
    {
        return fTimeBoundP;
    }

    public void setPTimeBound( TimeBound aTimeBoundP )
    {
        fTimeBoundP = aTimeBoundP;
    }
    
    public EventConstraint getPConstraint()
    {
        return fPConstraint;
    }

    public void setPConstraint( EventConstraint aPConstraint )
    {
        fPConstraint = aPConstraint;
    }

    public PrecedenceChainN1()
    {
        this( Event.getDefault(), Event.getDefault(), new ChainEvents(), null, null, null );
    }

    public PrecedenceChainN1( Event aEventP, Event aEventS, ChainEvents aEventTis,
                              TimeBound aTimeBoundP, EventConstraint aPConstraint, ProbabilityBound aProbBound ) 
    {
        super( aEventP, aEventS, aEventTis, aProbBound );

        fTimeBoundP = aTimeBoundP;
        fPConstraint = aPConstraint;
    }

    public int getType() 
    {
        return PSPConstants.P_PrecedenceChainN1;
    }

    public ArrayList<Event> getEvents() 
    {
        ArrayList<Event> Result = super.getEvents();

        for ( Event e : getTis().getEvents() )
        {
            if ( !Result.contains( e ) )
                    Result.add( e );
        }
        
        if ( fPConstraint != null )
        {
            if ( !Result.contains( fPConstraint.getEvent() ) )
                    Result.add( fPConstraint.getEvent() );
        }
        
        return Result;
    }

    public String getSpecificationAsSEL() 
    {
        StringBuilder sb = new StringBuilder();
        
        sb.append( "if " );
        sb.append( getP().getAsSELEvent() );
        sb.append( " [holds]" );
        
        sb.append( " then it must have been the case that " );
        sb.append( getS().getAsSELEvent() );
        sb.append( getTis().getSpecificationAsSEL( " and afterwards " ) );
        sb.append( " [have occurred]" );

        if ( fTimeBoundP != null )
        {
            sb.append( " " );
            sb.append( fTimeBoundP.getSpecificationAsSEL() );
        }

        if ( fPConstraint != null )
        {
            sb.append( " " );
            sb.append( fPConstraint.getSpecificationAsSEL() );
        }

        sb.append( " before " );
        sb.append( getP().getAsSELEvent() );
        sb.append( " [holds]" );

        if ( getProbabilityBound() != null )
        {
            sb.append( " " );
            sb.append( getProbabilityBound().getSpecificationAsSEL() );
        }
        
        return sb.toString();
    }    
}
