package se.got.ltl.visitors;

import se.got.ltl.LTLConjunction;
import se.got.ltl.LTLIDisjunction;
import se.got.ltl.LTLEventually;
import se.got.ltl.LTLIGlobally;
import se.got.ltl.LTLIIff;
import se.got.ltl.LTLIImplies;
import se.got.ltl.LTLINegation;
import se.got.ltl.LTLISince;
import se.got.ltl.LTLIUntil;
import se.got.ltl.LTLNext;
import se.got.ltl.atoms.LTLIPropositionalAtom;
import se.got.ltl.atoms.MITLIRelationalAtom;
import se.got.ltl.atoms.MITLITrue;

public interface MITLIVisitor<T> {

	public T visit(LTLIDisjunction formula);


	public T visit(MITLITrue formula);

	public T visit(LTLConjunction formula);

	public T visit(LTLINegation formula);

	public T visit(LTLIUntil formula);

	public T visit(LTLIImplies formula);

	public T visit(LTLIIff formula);

	public T visit(LTLISince formula);



	public T visit(LTLIPropositionalAtom formula);

	public T visit(MITLIRelationalAtom formula);


	public T visit(LTLEventually mitliEventually);


	public T visit(LTLIGlobally mitliGlobally);


	public T visit(LTLNext ltlNext);

}
