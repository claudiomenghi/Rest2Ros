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
package se.got.gui.util;

import javax.swing.JTextField;
import javax.swing.text.AttributeSet;
import javax.swing.text.BadLocationException;
import javax.swing.text.Document;
import javax.swing.text.PlainDocument;

public class TimeTextField extends JTextField 
{
    public TimeTextField()
    {
        super();
    }
    
    public TimeTextField( Document doc, String text, int columns )
    {
        super( doc, text, columns );
    }

    public TimeTextField( int cols ) 
    {
        super( cols );
    }

    public TimeTextField( String aText ) 
    {
        super( aText );
    }
    
    public TimeTextField( String text, int columns )
    {
        super( text, columns );
    }

    protected Document createDefaultModel() 
    {
        return new TimeDocument();
    }
 
    private static class TimeDocument extends PlainDocument 
    {
        public void insertString( int offs, String str, AttributeSet a ) throws BadLocationException
        {
            if (str == null) 
            {
                return;
            }
            
            StringBuilder lNumbers = new StringBuilder();
            char[] chars = str.toCharArray();
            int lLength = offs;
            
            for ( int i = 0; i < chars.length; i++ ) 
            {
                if ( lLength > 16 )
                    break;  // do not exceed 2^63-1 (by a mile)
                
                if ( Character.isDigit( chars[i] ) )
                {
                    lNumbers.append( chars[i] );
                    lLength++;
                }
                else
                    break; // add no more
            }
            
            super.insertString(offs, lNumbers.toString(), a);
         }
     }
 }
