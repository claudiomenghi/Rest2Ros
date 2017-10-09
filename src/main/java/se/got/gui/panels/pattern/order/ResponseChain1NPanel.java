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
import se.got.gui.dialogs.ProbabilityBoundDialog;
import se.got.gui.dialogs.TimeBoundDialog;
import se.got.gui.panels.pattern.PatternPanelFeatures;
import se.got.sel.Event;
import se.got.sel.patterns.Pattern;
import se.got.sel.patterns.order.ChainEvents;
import se.got.sel.patterns.order.ResponseChain1N;

public class ResponseChain1NPanel extends javax.swing.JPanel implements PatternPanelFeatures, ChainChanged
{
    private PSPController fPSPController;
    private ResponseChain1N fSelectedPattern;

    public Pattern getSelectedPattern()
    {
        return fSelectedPattern;
    }

    public void setSelectedPattern( Pattern aSelectedPattern ) 
    {
        if ( aSelectedPattern instanceof ResponseChain1N )
        {
            fSelectedPattern = (ResponseChain1N)aSelectedPattern;
            
            // update gui elements
            fP.setSelectedItem( fSelectedPattern.getP() );
            fS.setSelectedItem( fSelectedPattern.getS() );
            if ( fSelectedPattern.getSConstraint() != null )
                fZS.setSelectedItem( fSelectedPattern.getSConstraint().getEvent() );
            else
                fZS.setSelectedItem( Event.getConstraintDefault() );

            fTi.setChainSequence( fSelectedPattern.getTis().getTis() );
            
            if ( fSelectedPattern.getSTimeBound() == null )
                fTimeBound.setText( "Time Bound" );
            else
                fTimeBound.setText( fSelectedPattern.getSTimeBound().toString() );

            if ( fSelectedPattern.getProbabilityBound() == null )
                fProbabilityBound.setText( "Probability Bound" );
            else
                fProbabilityBound.setText( fSelectedPattern.getProbabilityBound().toString() );
        }
    }

    public ResponseChain1NPanel() 
    {
        initComponents();
        
        fSelectedPattern = new ResponseChain1N();
        fTi.setChainUpdateListener( this );
    }

    public void clearSelection()
    {
        setSelectedPattern( new ResponseChain1N() );
    }
    
    public void setController( PSPController aPSPController )
    {
        fPSPController = aPSPController;
        
        fP.setController( fPSPController );
        fS.setController( fPSPController );
        fZS.setController( fPSPController );
        fTi.setController( fPSPController );
    }

    public void updateEvents()
    {
        fP.updateEvents();
        fS.updateEvents();
        fZS.updateEvents();
        fTi.updateEvents();
    }

    public void updateChain( ChainEvents aEventChain )
    {
        fSelectedPattern.setTis( aEventChain );
        updatePattern();
    }

    private void updatePattern()
    {
        if ( fTi.getChainSequence().size() > 1 )
            fTiLabel.setText( "[eventually hold]" );
        else
            fTiLabel.setText( "[eventually holds]" );
            
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

    private void updateP()
    {
        fP.acceptSelection();

        fSelectedPattern.setP( (Event)fP.getSelectedItem() );

        updatePattern();
    }

    private void updateS()
    {
        fS.acceptSelection();

        fSelectedPattern.setS( (Event)fS.getSelectedItem() );

        updatePattern();
    }

    private void updateZS()
    {
        fZS.acceptSelection();

        fSelectedPattern.setSConstraint( EventConstraint.newEventConstraint( (Event)fZS.getSelectedItem() ) );

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
        fP = new se.got.gui.util.EventComboBox();
        fTiLabel = new javax.swing.JLabel();
        fLabel = new javax.swing.JLabel();
        fS = new se.got.gui.util.EventComboBox();
        fTimeBound = new javax.swing.JButton();
        fProbabilityBound = new javax.swing.JButton();
        fZS = new se.got.gui.util.ContraintComboBox();
        fLabel2 = new javax.swing.JLabel();
        fTi = new se.got.gui.panels.pattern.order.chains.ResponseChainEventSequencePanel();
        fLabel4 = new javax.swing.JLabel();

        setMaximumSize(new java.awt.Dimension(536, 350));
        setMinimumSize(new java.awt.Dimension(536, 276));

        fLabel1.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fLabel1.setText("if");

        fP.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fPActionPerformed(evt);
            }
        });

        fTiLabel.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fTiLabel.setText("[eventually holds]");

        fLabel.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fLabel.setText("then in response");

        fS.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fSActionPerformed(evt);
            }
        });

        fTimeBound.setText("Time Bound");
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

        fZS.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                fZSActionPerformed(evt);
            }
        });

        fLabel2.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fLabel2.setText("[has occurred]");

        fLabel4.setFont(new java.awt.Font("Lucida Grande", 1, 13)); // NOI18N
        fLabel4.setText("[eventually holds]");

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addGap(35, 35, 35)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                        .addComponent(fTi, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGroup(layout.createSequentialGroup()
                            .addComponent(fTiLabel)
                            .addGap(110, 110, 110)
                            .addComponent(fProbabilityBound, javax.swing.GroupLayout.PREFERRED_SIZE, 250, javax.swing.GroupLayout.PREFERRED_SIZE)))
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(fLabel1)
                        .addGap(18, 18, 18)
                        .addComponent(fP, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(fLabel2)
                        .addGap(205, 205, 205))
                    .addGroup(javax.swing.GroupLayout.Alignment.LEADING, layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                        .addGroup(layout.createSequentialGroup()
                            .addComponent(fTimeBound, javax.swing.GroupLayout.PREFERRED_SIZE, 250, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addGap(18, 18, 18)
                            .addComponent(fZS, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addGroup(layout.createSequentialGroup()
                            .addComponent(fLabel)
                            .addGap(18, 18, 18)
                            .addComponent(fS, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addGap(18, 18, 18)
                            .addComponent(fLabel4))))
                .addContainerGap(20, Short.MAX_VALUE))
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(fLabel1)
                    .addComponent(fP, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(fLabel2))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(fLabel)
                    .addComponent(fS, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                    .addComponent(fLabel4))
                .addGap(18, 18, 18)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(fTimeBound)
                    .addComponent(fZS, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addComponent(fTi, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addGap(18, 18, 18)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(fTiLabel)
                    .addComponent(fProbabilityBound))
                .addContainerGap(80, Short.MAX_VALUE))
        );
    }// </editor-fold>//GEN-END:initComponents

    private void fTimeBoundActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_fTimeBoundActionPerformed
        TimeBoundDialog lDialog = new TimeBoundDialog( fPSPController );

        TimeBound lNewTimeBound = lDialog.showDialog( fSelectedPattern.getS(), fSelectedPattern.getSTimeBound() );

        if ( lNewTimeBound == null )
            fTimeBound.setText( "Interval Time Bound" );
        else
            fTimeBound.setText( lNewTimeBound.toString() );

        fSelectedPattern.setSTimeBound( lNewTimeBound );
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

    private void fPActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_fPActionPerformed
        // check that selected Event is not used in pattern
        Event lEvent = (Event)fP.getSelectedItem();
        
        if ( lEvent != null )
        {
            if ( fP.isStable( lEvent ) || isEventSelectionPossible( lEvent ) )
            {
                updateP();
            }
            else
                fP.revertSelection();
        }
    }//GEN-LAST:event_fPActionPerformed

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

    private void fZSActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_fZSActionPerformed
        // check that selected Event is not used in pattern
        Event lEvent = (Event)fZS.getSelectedItem();
        
        if ( lEvent != null )
        {
            if ( fZS.isStable( lEvent ) || isEventSelectionPossible( lEvent ) )
            {
                updateZS();
            }
            else
                fZS.revertSelection();
        }
    }//GEN-LAST:event_fZSActionPerformed

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JLabel fLabel;
    private javax.swing.JLabel fLabel1;
    private javax.swing.JLabel fLabel2;
    private javax.swing.JLabel fLabel4;
    private se.got.gui.util.EventComboBox fP;
    private javax.swing.JButton fProbabilityBound;
    private se.got.gui.util.EventComboBox fS;
    private se.got.gui.panels.pattern.order.chains.ResponseChainEventSequencePanel fTi;
    private javax.swing.JLabel fTiLabel;
    private javax.swing.JButton fTimeBound;
    private se.got.gui.util.ContraintComboBox fZS;
    // End of variables declaration//GEN-END:variables

}
