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
package se.got.gui.panels.pattern.order;

import se.got.constraints.EventConstraint;
import se.got.constraints.ProbabilityBound;
import se.got.constraints.TimeBound;
import se.got.engine.PSPController;
import se.got.gui.dialogs.IntervalTimeBoundDialog;
import se.got.gui.dialogs.ProbabilityBoundDialog;
import se.got.gui.dialogs.TimeBoundDialog;
import se.got.gui.panels.pattern.PatternPanelFeatures;
import se.got.sel.Event;
import se.got.sel.patterns.Pattern;
import se.got.sel.patterns.order.ChainEvents;
import se.got.sel.patterns.order.PrecedenceChainN1;

public class PrecedenceChainN1Panel extends javax.swing.JPanel implements PatternPanelFeatures, ChainChanged
{
    private PSPController fPSPController;
    private PrecedenceChainN1 fSelectedPattern;

    public Pattern getSelectedPattern()
    {
        return fSelectedPattern;
    }

    public void setSelectedPattern( Pattern aSelectedPattern ) 
    {
        if ( aSelectedPattern instanceof PrecedenceChainN1 )
        {
            fSelectedPattern = (PrecedenceChainN1)aSelectedPattern;
            
            // update gui elements
            fP1.setSelectedItem( fSelectedPattern.getP() );
            fP2.setSelectedItem( fSelectedPattern.getP() );
            fS.setSelectedItem( fSelectedPattern.getS() );
            if ( fSelectedPattern.getPConstraint() != null )
                fZP.setSelectedItem( fSelectedPattern.getPConstraint().getEvent() );
            else
                fZP.setSelectedItem( Event.getConstraintDefault() );

            fTi.setChainSequence( fSelectedPattern.getTis().getTis() );
            
            if ( fSelectedPattern.getPTimeBound() == null )
                fTimeBound.setText( "Time Bound" );
            else
                fTimeBound.setText( fSelectedPattern.getPTimeBound().toString() );

            if ( fSelectedPattern.getProbabilityBound() == null )
                fProbabilityBound.setText( "Probability Bound" );
            else
                fProbabilityBound.setText( fSelectedPattern.getProbabilityBound().toString() );
        }
    }

    public PrecedenceChainN1Panel() 
    {
        initComponents();
        
        fSelectedPattern = new PrecedenceChainN1();
        fTi.setChainUpdateListener( this );
    }

    public void clearSelection()
    {
        setSelectedPattern( new PrecedenceChainN1() );
    }
    
    public void setController( PSPController aPSPController )
    {
        fPSPController = aPSPController;
        
        fP1.setController( fPSPController );
        fP2.setController( fPSPController );
        fS.setController( fPSPController );
        fZP.setController( fPSPController );
        fTi.setController( fPSPController );
    }

    public void updateEvents()
    {
        fP1.updateEvents();
        fP2.updateEvents();
        fS.updateEvents();
        fZP.updateEvents();
        fTi.updateEvents();
    }

    public void updateChain( ChainEvents aEventChain )
    {
        fSelectedPattern.setTis( aEventChain );
        updatePattern();
    }

    private void updatePattern()
    {
        if ( fPSPController != null )
            fPSPController.updatePattern();
    }

    private boolean isEventSelectionPossible( Event aEvent )
    {
        if ( fPSPController != null )
            return fPSPController.isPatternEventSelectionPossible( aEvent );

        return true;
    }

    private boolean isSSelectionPossible( Event aS, Event aSAlt )
    {
        if ( fPSPController != null )
            return fPSPController.isPatternEventSelectionPossible( aS, aSAlt );

        return true;
    }

    private void updateP1()
    {
        fP1.acceptSelection();
        fP2.setSelectedItem( fP1.getSelectedItem() );
        fP2.acceptSelection();

        fSelectedPattern.setP( (Event)fP1.getSelectedItem() );

        updatePattern();
    }

    private void updateP2()
    {
        fP2.acceptSelection();
        fP1.setSelectedItem( fP2.getSelectedItem() );
        fP1.acceptSelection();

        fSelectedPattern.setP( (Event)fP2.getSelectedItem() );

        updatePattern();
    }

    private void updateS()
    {
        fS.acceptSelection();

        fSelectedPattern.setS( (Event)fS.getSelectedItem() );

        updatePattern();
    }

    private void updateZP()
    {
        fZP.acceptSelection();

        fSelectedPattern.setPConstraint( EventConstraint.newEventConstraint( (Event)fZP.getSelectedItem() ) );

        updatePattern();
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        fLabel1 = new javax.swing.JLabel();
        fP1 = new se.got.gui.util.EventComboBox();
        fTiLabel = new javax.swing.JLabel();
        fTi = new se.got.gui.panels.pattern.order.chains.PrecedenceChainEventSequencePanel();
        fLabel4 = new javax.swing.JLabel();
        fS = new se.got.gui.util.EventComboBox();
        fTimeBound = new javax.swing.JButton();
        fProbabilityBound = new javax.swing.JButton();
        fLabel6 = new javax.swing.JLabel();
        fP2 = new se.got.gui.util.EventComboBox();
        fLabel7 = new javax.swing.JLabel();
        fZP = new se.got.gui.util.ContraintComboBox();
        fLabel2 = new javax.swing.JLabel();

        setMaximumSize(new java.awt.Dimension(536, 350));
        setMinimumSize(new java.awt.Dimension(536, 276));

        fLabel1.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fLabel1.setText("if");

        fP1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fP1ActionPerformed(evt);
            }
        });

        fTiLabel.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fTiLabel.setText("[have occurred]");

        fTi.setMaximumSize(new java.awt.Dimension(504, 200));
        fTi.setMinimumSize(new java.awt.Dimension(504, 92));

        fLabel4.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fLabel4.setText("then it must be the case that");

        fS.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fSActionPerformed(evt);
            }
        });

        fTimeBound.setText("Interval Time Bound");
        fTimeBound.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fTimeBoundActionPerformed(evt);
            }
        });

        fProbabilityBound.setText("Probability Bound");
        fProbabilityBound.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fProbabilityBoundActionPerformed(evt);
            }
        });

        fLabel6.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fLabel6.setText("before");

        fP2.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fP2ActionPerformed(evt);
            }
        });

        fLabel7.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fLabel7.setText("[holds]");

        fZP.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fZPActionPerformed(evt);
            }
        });

        fLabel2.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fLabel2.setText("[holds]");

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addGap(35, 35, 35)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(fTi, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(fProbabilityBound, javax.swing.GroupLayout.PREFERRED_SIZE, 250, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addGroup(layout.createSequentialGroup()
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addComponent(fTiLabel)
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(fLabel1)
                                .addGap(18, 18, 18)
                                .addComponent(fP1, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addGap(18, 18, 18)
                                .addComponent(fLabel2))
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(fLabel4)
                                .addGap(18, 18, 18)
                                .addComponent(fS, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(fTimeBound, javax.swing.GroupLayout.PREFERRED_SIZE, 250, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addGap(18, 18, 18)
                                .addComponent(fZP, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(fLabel6)
                                .addGap(18, 18, 18)
                                .addComponent(fP2, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addGap(18, 18, 18)
                                .addComponent(fLabel7)))
                        .addGap(48, 48, 48)))
                .addContainerGap(52, Short.MAX_VALUE))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(fLabel1)
                    .addComponent(fP1, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(fLabel2))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(fLabel4)
                    .addComponent(fS, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(fTi, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addComponent(fTiLabel)
                .addGap(18, 18, 18)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(fTimeBound)
                    .addComponent(fZP, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addGap(18, 18, 18)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(fLabel6)
                    .addComponent(fP2, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(fLabel7))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(fProbabilityBound)
                .addContainerGap(19, Short.MAX_VALUE))
        );
    }// </editor-fold>//GEN-END:initComponents

    private void fTimeBoundActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_fTimeBoundActionPerformed
        TimeBoundDialog lDialog = new IntervalTimeBoundDialog( fPSPController );

        TimeBound lNewTimeBound = lDialog.showDialog( fSelectedPattern.getS(), fSelectedPattern.getPTimeBound() );

        if ( lNewTimeBound == null )
            fTimeBound.setText( "Interval Time Bound" );
        else
            fTimeBound.setText( lNewTimeBound.toString() );

        fSelectedPattern.setPTimeBound( lNewTimeBound );
        updatePattern();
    }//GEN-LAST:event_fTimeBoundActionPerformed

    private void fProbabilityBoundActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_fProbabilityBoundActionPerformed
        ProbabilityBoundDialog lDialog = new ProbabilityBoundDialog( fPSPController );

        ProbabilityBound lNewProbabilityBound = lDialog.showDialog( fSelectedPattern.getProbabilityBound() );

        if ( lNewProbabilityBound == null )
            fProbabilityBound.setText( "Probability Bound" );
        else
            fProbabilityBound.setText( lNewProbabilityBound.toString() );

        fSelectedPattern.setProbabilityBound( lNewProbabilityBound );
        updatePattern();
    }//GEN-LAST:event_fProbabilityBoundActionPerformed

    private void fP1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_fP1ActionPerformed
        // check that selected Event is not used in pattern
        Event lEvent = (Event)fP1.getSelectedItem();
        
        if ( lEvent != null )
        {
            if ( fP1.isStable( lEvent ) || isSSelectionPossible( lEvent, (Event)fP2.getSelectedItem() ) )
            {
                fP1.acceptSelection();
                updateP1();
            }
            else
                fP1.revertSelection();
        }
    }//GEN-LAST:event_fP1ActionPerformed

    private void fP2ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_fP2ActionPerformed
        // check that selected Event is not used in pattern
        Event lEvent = (Event)fP2.getSelectedItem();
        
        if ( lEvent != null )
        {
            if ( fP2.isStable( lEvent ) || isSSelectionPossible( lEvent, (Event)fP1.getSelectedItem() ) )
            {
                updateP2();
            }
            else
                fP2.revertSelection();
        }
    }//GEN-LAST:event_fP2ActionPerformed

    private void fSActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_fSActionPerformed
        // check that selected Event is not used in pattern
        Event lEvent = (Event)fS.getSelectedItem();
        
        if ( lEvent != null )
        {
            if ( fS.isStable( lEvent ) || isEventSelectionPossible( lEvent ) )
            {
                updateS();
            }
            else
                fS.revertSelection();
        }
    }//GEN-LAST:event_fSActionPerformed

    private void fZPActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_fZPActionPerformed
        // check that selected Event is not used in pattern
        Event lEvent = (Event)fZP.getSelectedItem();
        
        if ( lEvent != null )
        {
            if ( fZP.isStable( lEvent ) || isEventSelectionPossible( lEvent ) )
            {
                updateZP();
            }
            else
                fZP.revertSelection();
        }
    }//GEN-LAST:event_fZPActionPerformed

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JLabel fLabel1;
    private javax.swing.JLabel fLabel2;
    private javax.swing.JLabel fLabel4;
    private javax.swing.JLabel fLabel6;
    private javax.swing.JLabel fLabel7;
    private se.got.gui.util.EventComboBox fP1;
    private se.got.gui.util.EventComboBox fP2;
    private javax.swing.JButton fProbabilityBound;
    private se.got.gui.util.EventComboBox fS;
    private se.got.gui.panels.pattern.order.chains.PrecedenceChainEventSequencePanel fTi;
    private javax.swing.JLabel fTiLabel;
    private javax.swing.JButton fTimeBound;
    private se.got.gui.util.ContraintComboBox fZP;
    // End of variables declaration//GEN-END:variables

}
